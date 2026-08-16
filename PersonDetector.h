#pragma once

#include <mutex>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <torch/script.h>
#include <torch/torch.h>

// Qt 의존성 없음 - RPi4 등 Qt 없는 환경에서 그대로 빌드 가능.
// viewer/VideoThread.h(Qt 뷰어 전용)의 letterbox/decode_yolo_output/nms 로직을
// 그대로 옮기되, Qt(QMutex, QThread, 시그널 등)만 std::mutex 등 표준 라이브러리로
// 교체했다. 두 파일의 탐지 알고리즘(letterbox 방식, 디코드 수식, NMS 임계값)은
// 반드시 같은 값을 유지해야 한다 - 한쪽만 고치면 같은 모델인데 결과가 달라진다.
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
    // modelPath: TorchScript(.pt) 경로. CPU 전용으로 로드한다(map_location=CPU) -
    // CUDA에서 export된 모델도 device 인자 없이 로드하면 CPU 전용 환경(RPi4)에서
    // "Could not run aten::empty_strided ... CUDA backend"로 실패하므로 반드시 필요함.
    // inputSize: letterbox 정사각형 크기. 모델 학습/export 시 imgsz와 반드시 일치해야 함.
    explicit PersonDetector(const std::string& modelPath, int inputSize = 320,
        float confThreshold = 0.3f, float nmsThreshold = 0.45f);

    // 생성자에서 모델 로드가 실패해도 예외를 던지지 않는다(내부에서 흡수).
    // 호출부는 반드시 이 값을 확인한 뒤 detect()를 호출할 것.
    bool isLoaded() const { return loaded_; }

    // frame 1장에서 사람을 탐지한다. 반환 좌표는 letterbox 좌표가 아니라
    // 원본 frame 픽셀 좌표계로 이미 역변환된 값이다.
    // drawBoxes가 true면 frame에 사각형+confidence 텍스트를 직접 그린다(in-place).
    std::vector<DetectionBox> detect(cv::Mat& frame, bool drawBoxes = true);

    // 박스 목록 -> 발 위치 목록. x는 박스 중앙, y는 박스 하단(ymax).
    static std::vector<FootPoint> getFootPoints(const std::vector<DetectionBox>& boxes);

    // 여러 스레드가 같은 PersonDetector 인스턴스를 공유해 detect()를 동시에 부를 때
    // 보호가 필요하면 이 뮤텍스를 쓰면 된다(내부 forward 호출에도 이미 적용돼 있음).
    // 스레드마다 인스턴스를 따로 두면(권장) 이건 신경 쓸 필요 없음.
    std::mutex& inferenceMutex() { return modelMutex_; }

private:
    struct LetterboxInfo {
        float scale;
        int padX;
        int padY;
    };

    static cv::Mat letterbox(const cv::Mat& src, int targetSize, LetterboxInfo& info);
    static float calculateIou(const DetectionBox& a, const DetectionBox& b);
    static std::vector<DetectionBox> nms(std::vector<DetectionBox> boxes, float iouThreshold);
    static std::vector<DetectionBox> decodeOutput(torch::Tensor output, float confThreshold);

    torch::jit::script::Module model_;
    bool loaded_ = false;
    int inputSize_;
    float confThreshold_;
    float nmsThreshold_;
    std::mutex modelMutex_;
};

}  // namespace detection