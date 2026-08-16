#include "PersonDetector.h"

#include <algorithm>
#include <iostream>

namespace detection {

PersonDetector::PersonDetector(const std::string& modelPath, int inputSize,
    float confThreshold, float nmsThreshold)
    : inputSize_(inputSize), confThreshold_(confThreshold), nmsThreshold_(nmsThreshold) {
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

// YOLOv8 letterbox: 종횡비를 유지한 채 targetSize 정사각형에 맞춰 리사이즈하고
// 남는 부분은 회색(114,114,114)으로 패딩. scale/pad는 박스 좌표를 원본 프레임으로
// 되돌릴 때 필요해서 함께 반환함. VideoThread.h::letterbox와 동일 수식.
cv::Mat PersonDetector::letterbox(const cv::Mat& src, int targetSize, LetterboxInfo& info) {
    int w = src.cols, h = src.rows;
    float scale = std::min((float)targetSize / w, (float)targetSize / h);
    int newW = static_cast<int>(std::round(w * scale));
    int newH = static_cast<int>(std::round(h * scale));

    cv::Mat resized;
    cv::resize(src, resized, cv::Size(newW, newH));

    cv::Mat out(targetSize, targetSize, src.type(), cv::Scalar(114, 114, 114));
    int padX = (targetSize - newW) / 2;
    int padY = (targetSize - newH) / 2;
    resized.copyTo(out(cv::Rect(padX, padY, newW, newH)));

    info.scale = scale;
    info.padX = padX;
    info.padY = padY;
    return out;
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

// model.export(format='torchscript', nms=False)의 출력은 (1,5,N) = 채널별
// [cx,cy,w,h,score] (letterbox 좌표계, 픽셀 단위, score는 이미 sigmoid 적용된
// person 클래스 확신도 - 클래스가 1개뿐이라 별도 argmax 불필요).
// nms=True 버전은 torchvision::nms C++ 연산에 의존해서 RPi4용 LibTorch만으로는
// 링크가 안 되기 때문에 raw 출력 + 아래의 가벼운 NMS 조합으로 대체함.
// VideoThread.h::decode_yolo_output과 완전히 동일한 수식 - 한쪽만 고치면
// 같은 모델인데 결과가 달라지므로 반드시 같이 수정할 것.
std::vector<DetectionBox> PersonDetector::decodeOutput(torch::Tensor output, float confThreshold) {
    std::vector<DetectionBox> boxes;
    output = output.cpu();
    if (output.dim() == 3) {
        output = output[0];
    }

    auto acc = output.accessor<float, 2>();
    int n = static_cast<int>(output.size(1));
    for (int i = 0; i < n; ++i) {
        float score = acc[4][i];
        if (score < confThreshold) {
            continue;
        }

        float cx = acc[0][i];
        float cy = acc[1][i];
        float w = acc[2][i];
        float h = acc[3][i];

        boxes.push_back({ cx - w / 2.0f, cy - h / 2.0f, cx + w / 2.0f, cy + h / 2.0f, score });
    }

    return boxes;
}

std::vector<DetectionBox> PersonDetector::detect(cv::Mat& frame, bool drawBoxes) {
    if (!loaded_ || frame.empty()) {
        return {};
    }

    int h = frame.rows;
    int w = frame.cols;

    LetterboxInfo lb;
    cv::Mat imgResized = letterbox(frame, inputSize_, lb);
    cv::cvtColor(imgResized, imgResized, cv::COLOR_BGR2RGB);
    imgResized.convertTo(imgResized, CV_32FC3, 1.0 / 255.0);

    torch::Tensor imgTensor = torch::from_blob(
        imgResized.data, { 1, inputSize_, inputSize_, 3 }, torch::kFloat32);
    imgTensor = imgTensor.permute({ 0, 3, 1, 2 }).contiguous();

    torch::Tensor output;
    try {
        std::lock_guard<std::mutex> lock(modelMutex_);
        torch::NoGradGuard noGrad;
        auto outputs = model_.forward({ imgTensor });
        output = outputs.toTensor();
    }
    catch (const std::exception& e) {
        std::cerr << "[PersonDetector] 추론 실패: " << e.what() << std::endl;
        return {};
    }

    auto boxes = decodeOutput(output, confThreshold_);
    boxes = nms(std::move(boxes), nmsThreshold_);

    std::vector<DetectionBox> result;
    result.reserve(boxes.size());
    for (const auto& box : boxes) {
        // letterbox 좌표계 -> 원본 프레임 좌표계로 역변환
        float xmin = std::max(0.0f, std::min((box.xmin - lb.padX) / lb.scale, (float)w));
        float ymin = std::max(0.0f, std::min((box.ymin - lb.padY) / lb.scale, (float)h));
        float xmax = std::max(0.0f, std::min((box.xmax - lb.padX) / lb.scale, (float)w));
        float ymax = std::max(0.0f, std::min((box.ymax - lb.padY) / lb.scale, (float)h));

        result.push_back({ xmin, ymin, xmax, ymax, box.conf });

        if (drawBoxes) {
            cv::rectangle(frame, cv::Point((int)xmin, (int)ymin), cv::Point((int)xmax, (int)ymax),
                cv::Scalar(0, 255, 0), 3);
            std::string confStr = cv::format("%.2f", box.conf);
            cv::putText(frame, confStr, cv::Point((int)xmin, std::max(20, (int)ymin - 10)),
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