#include "PersonDetector.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace detection {

PersonDetector::PersonDetector(const std::string& modelPath, int imgWidth, int imgHeight,
    float confThreshold, float nmsThreshold)
    : imgWidth_(imgWidth), imgHeight_(imgHeight),
    confThreshold_(confThreshold), nmsThreshold_(nmsThreshold) {
    try {
        model_ = torch::jit::load(modelPath, torch::kCPU);
        model_.eval();
        loaded_ = true;
    }
    catch (const c10::Error& e) {
        std::cerr << "[PersonDetector] 모델 로드 실패: " << e.what() << std::endl;
        loaded_ = false;
    }
}

float PersonDetector::calculateIou(const DetectionBox& a, const DetectionBox& b) {
    float x1 = std::max(a.xmin, b.xmin);
    float y1 = std::max(a.ymin, b.ymin);
    float x2 = std::min(a.xmax, b.xmax);
    float y2 = std::min(a.ymax, b.ymax);

    float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
    float area1 = (a.xmax - a.xmin) * (a.ymax - a.ymin);
    float area2 = (b.xmax - b.xmin) * (b.ymax - b.ymin);
    float unionVal = area1 + area2 - inter;

    return inter / (unionVal + 1e-6f);
}

std::vector<DetectionBox> PersonDetector::nms(std::vector<DetectionBox> boxes, float iouThreshold) {
    if (boxes.empty()) return {};

    std::sort(boxes.begin(), boxes.end(), [](const DetectionBox& a, const DetectionBox& b) {
        return a.conf > b.conf;
        });

    std::vector<DetectionBox> keep;
    while (!boxes.empty()) {
        DetectionBox best = boxes.front();
        boxes.erase(boxes.begin());
        keep.push_back(best);

        std::vector<DetectionBox> remain;
        for (const auto& box : boxes) {
            if (calculateIou(best, box) < iouThreshold) {
                remain.push_back(box);
            }
        }
        boxes = remain;
    }
    return keep;
}

// train_person_detector.py::decode_prediction과 완전히 동일한 수식.
// raw grid 출력(obj_logit, tx_logit, ty_logit, tw_log, th_log)을 sigmoid/exp로
// 복원한다. 한쪽만 고치면 같은 모델인데 결과가 달라지므로 Python 쪽과 반드시 같이 볼 것.
//
//   obj = sigmoid(pred[0])                       -> objectness/confidence
//   tx  = sigmoid(pred[1]) * 2 - 0.5              -> 다중 양성 셀 할당으로 [-0.5,1.5) 범위
//   ty  = sigmoid(pred[2]) * 2 - 0.5
//   tw  = clamp(pred[3], max=4.0)                 -> WH_LOG_CLAMP (exp 오버플로 방지)
//   th  = clamp(pred[4], max=4.0)
//   bw  = exp(tw) * img_w,  bh = exp(th) * img_h  -> log(bw/W) 인코딩의 정확한 역함수
//   cx  = (grid_x + tx) * cell_w,  cy = (grid_y + ty) * cell_h
std::vector<DetectionBox> PersonDetector::decodePrediction(
    const torch::Tensor& predIn, int imgW, int imgH, float confThreshold) {
    std::vector<DetectionBox> boxes;
    torch::Tensor pred = predIn.detach().cpu().contiguous();
    // pred: (5, gridH, gridW)
    int64_t gridH = pred.size(1);
    int64_t gridW = pred.size(2);
    float cellW = static_cast<float>(imgW) / static_cast<float>(gridW);
    float cellH = static_cast<float>(imgH) / static_cast<float>(gridH);

    auto predAcc = pred.accessor<float, 3>();
    constexpr float kWhLogClamp = 4.0f;  // train_person_detector.py::WH_LOG_CLAMP

    auto sigmoidf = [](float v) { return 1.0f / (1.0f + std::exp(-v)); };

    for (int64_t y = 0; y < gridH; ++y) {
        for (int64_t x = 0; x < gridW; ++x) {
            float objScore = sigmoidf(predAcc[0][y][x]);
            if (objScore <= confThreshold) {
                continue;
            }

            float tx = sigmoidf(predAcc[1][y][x]) * 2.0f - 0.5f;
            float ty = sigmoidf(predAcc[2][y][x]) * 2.0f - 0.5f;
            float tw = std::min(predAcc[3][y][x], kWhLogClamp);
            float th = std::min(predAcc[4][y][x], kWhLogClamp);

            float bw = std::exp(tw) * static_cast<float>(imgW);
            float bh = std::exp(th) * static_cast<float>(imgH);

            float cx = (static_cast<float>(x) + tx) * cellW;
            float cy = (static_cast<float>(y) + ty) * cellH;

            float xmin = std::max(0.0f, cx - bw / 2.0f);
            float ymin = std::max(0.0f, cy - bh / 2.0f);
            float xmax = std::min(static_cast<float>(imgW), cx + bw / 2.0f);
            float ymax = std::min(static_cast<float>(imgH), cy + bh / 2.0f);

            boxes.push_back({ xmin, ymin, xmax, ymax, objScore });
        }
    }
    return boxes;
}

// FPN(9단계 등)처럼 출력이 레벨별 텐서 tuple/list일 수도 있으므로 IValue로 받아
// 분기한다. 단일 스케일(11단계)이면 텐서 하나라 첫 분기만 탄다.
std::vector<DetectionBox> PersonDetector::decodeOutput(
    const c10::IValue& output, int imgW, int imgH, float confThreshold) {
    std::vector<DetectionBox> boxes;

    auto decodeOne = [&](torch::Tensor t) {
        if (t.dim() == 4) {
            t = t[0];  // 배치 차원 제거 -> (5, gridH, gridW)
        }
        auto part = decodePrediction(t, imgW, imgH, confThreshold);
        boxes.insert(boxes.end(), part.begin(), part.end());
        };

    if (output.isTensor()) {
        decodeOne(output.toTensor());
    }
    else if (output.isTuple()) {
        for (const auto& elem : output.toTuple()->elements()) {
            decodeOne(elem.toTensor());
        }
    }
    else if (output.isTensorList()) {
        for (const auto& t : output.toTensorList()) {
            decodeOne(t);
        }
    }
    else {
        std::cerr << "[PersonDetector] 알 수 없는 모델 출력 타입입니다.\n";
    }

    return boxes;
}

std::vector<DetectionBox> PersonDetector::detect(cv::Mat& frame, bool drawBoxes) {
    if (!loaded_ || frame.empty()) {
        return {};
    }

    int origW = frame.cols;
    int origH = frame.rows;

    // 11단계 등 커스텀 모델은 letterbox가 아니라 종횡비 유지 없는 단순 resize.
    // train_person_detector.py::PersonDataset._load_resized와 동일한 전처리.
    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(imgWidth_, imgHeight_));
    cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
    resized.convertTo(resized, CV_32FC3, 1.0 / 255.0);
    // custom 백본은 ImageNet 정규화를 쓰지 않는다(랜덤 초기화로 학습되어 0~1 그대로 씀).
    // mobilenet 백본 모델(9단계 등)을 나중에 붙이면 여기에 정규화 분기가 필요함 -
    // PersonDataset.normalize / detect_video.py의 --normalize 플래그 참고.

    torch::Tensor imgTensor = torch::from_blob(
        resized.data, { 1, imgHeight_, imgWidth_, 3 }, torch::kFloat32);
    imgTensor = imgTensor.permute({ 0, 3, 1, 2 }).contiguous();

    c10::IValue output;
    try {
        std::lock_guard<std::mutex> lock(modelMutex_);
        torch::NoGradGuard noGrad;
        output = model_.forward({ imgTensor });
    }
    catch (const std::exception& e) {
        std::cerr << "[PersonDetector] 추론 실패: " << e.what() << std::endl;
        return {};
    }

    auto boxes = decodeOutput(output, imgWidth_, imgHeight_, confThreshold_);
    boxes = nms(std::move(boxes), nmsThreshold_);

    std::vector<DetectionBox> result;
    result.reserve(boxes.size());
    float sx = static_cast<float>(origW) / static_cast<float>(imgWidth_);
    float sy = static_cast<float>(origH) / static_cast<float>(imgHeight_);

    for (const auto& box : boxes) {
        // 단순 resize 좌표계 -> 원본 프레임 좌표계. letterbox와 달리 pad 보정이
        // 없고, 종횡비를 유지하지 않고 늘렸으므로 x/y 배율(sx, sy)이 다를 수 있다.
        float xmin = std::max(0.0f, std::min(box.xmin * sx, static_cast<float>(origW)));
        float ymin = std::max(0.0f, std::min(box.ymin * sy, static_cast<float>(origH)));
        float xmax = std::max(0.0f, std::min(box.xmax * sx, static_cast<float>(origW)));
        float ymax = std::max(0.0f, std::min(box.ymax * sy, static_cast<float>(origH)));

        result.push_back({ xmin, ymin, xmax, ymax, box.conf });

        if (drawBoxes) {
            cv::rectangle(frame, cv::Point(static_cast<int>(xmin), static_cast<int>(ymin)),
                cv::Point(static_cast<int>(xmax), static_cast<int>(ymax)),
                cv::Scalar(0, 255, 0), 3);
            std::string confStr = cv::format("%.2f", box.conf);
            cv::putText(frame, confStr,
                cv::Point(static_cast<int>(xmin), std::max(20, static_cast<int>(ymin) - 10)),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        }
    }

    return result;
}

std::vector<FootPoint> PersonDetector::getFootPoints(const std::vector<DetectionBox>& boxes) {
    std::vector<FootPoint> points;
    points.reserve(boxes.size());
    for (const auto& box : boxes) {
        points.push_back({ (box.xmin + box.xmax) / 2.0f, box.ymax, box.conf });
    }
    return points;
}

}  // namespace detection