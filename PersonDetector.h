#pragma once

#include <mutex>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <torch/script.h>
#include <torch/torch.h>

// hi/01_custom_model/train_person_detector.py의 grid 기반 커스텀 모델(11단계 lite 등)
// 전용. 이전 YOLOv8 기반 PersonDetector는 더 이상 쓰지 않기로 해서 이 클래스로
// 덮어썼다 - YOLOv8 버전이 필요하면 git 히스토리에서 복구할 것.
//
// 전처리: letterbox(정사각 패딩)가 아니라 단순 stretch resize(종횡비 유지 안 함) +
// raw grid (1,5,H,W) 출력을 sigmoid/exp로 직접 복원.
// 디코드 수식은 train_person_detector.py::decode_prediction과 반드시 동일하게
// 유지해야 한다 - 한쪽만 고치면 같은 모델인데 결과가 달라진다.
namespace detection {

// DetectionBox라는 이름이 VideoThread.h에도 전역으로 있어서, 두 헤더를 같은
// 번역 단위에서 함께 include하면 이름이 겹친다. 네임스페이스로 감싸 그 위험을 없앴다.
struct DetectionBox {
    float xmin, ymin, xmax, ymax, conf;
};

// 보행자의 지면 접점(발 위치) 픽셀 좌표. 호모그래피로 위성지도에 투영할 때 이 점을 쓴다.
struct FootPoint {
    float x, y;
    float conf;
};

class PersonDetector {
public:
    // modelPath: hi/01_custom_model에서 CPU로 export한 TorchScript(.pt).
    // imgWidth/imgHeight: 학습 시 입력 크기와 반드시 일치해야 함(기본 320x240, 11단계 lite 기준).
    // confThreshold/nmsThreshold: 학습 스크립트가 evaluate()에서 쓰던 기본값
    // (conf 0.05~0.25, NMS IoU 0.3) 기준.
    explicit PersonDetector(const std::string& modelPath, int imgWidth = 320, int imgHeight = 240,
        float confThreshold = 0.25f, float nmsThreshold = 0.3f);

    // 생성자에서 모델 로드가 실패해도 예외를 던지지 않는다(내부에서 흡수).
    // 호출부는 반드시 이 값을 확인한 뒤 detect()를 호출할 것.
    bool isLoaded() const { return loaded_; }

    // frame 1장에서 사람을 탐지한다. 반환 좌표는 원본 frame 픽셀 좌표계.
    // drawBoxes가 true면 frame에 사각형+confidence 텍스트를 직접 그린다(in-place).
    std::vector<DetectionBox> detect(cv::Mat& frame, bool drawBoxes = true);

    // 박스 목록 -> 발 위치 목록. x는 박스 중앙, y는 박스 하단(ymax).
    static std::vector<FootPoint> getFootPoints(const std::vector<DetectionBox>& boxes);

    // 여러 스레드가 같은 PersonDetector 인스턴스를 공유해 detect()를 동시에 부를 때
    // 보호가 필요하면 이 뮤텍스를 쓰면 된다(내부 forward 호출에도 이미 적용돼 있음).
    // 스레드마다 인스턴스를 따로 두면(권장) 이건 신경 쓸 필요 없음.
    std::mutex& inferenceMutex() { return modelMutex_; }

private:
    static float calculateIou(const DetectionBox& a, const DetectionBox& b);
    static std::vector<DetectionBox> nms(std::vector<DetectionBox> boxes, float iouThreshold);

    // 텐서 1개(단일 스케일)에서 그리드 디코드.
    // train_person_detector.py::decode_prediction과 완전히 동일한 수식.
    static std::vector<DetectionBox> decodePrediction(
        const torch::Tensor& pred, int imgW, int imgH, float confThreshold);

    // FPN(9단계 등)처럼 출력이 레벨별 텐서 tuple/list일 수도 있어 IValue로 받아 분기.
    // 단일 스케일(11단계)이면 텐서 하나라 첫 분기만 탄다.
    static std::vector<DetectionBox> decodeOutput(
        const c10::IValue& output, int imgW, int imgH, float confThreshold);

    torch::jit::script::Module model_;
    bool loaded_ = false;
    int imgWidth_;
    int imgHeight_;
    float confThreshold_;
    float nmsThreshold_;
    std::mutex modelMutex_;
};

}  // namespace detection