# h2au

RC카 보행자 탐지 + GPS 위성지도 프로젝트 (RPi4 배포용).

## 빌드 (RPi4 / Linux)

### 의존성

- CMake >= 3.14, C++17 컴파일러
- OpenCV (`libopencv-dev`)
- ONNX Runtime C/C++ SDK — **공식 CMake 패키지가 없어 압축 해제 경로를 직접 지정해야 함**

### ONNX Runtime 설치

[릴리스 목록](https://github.com/microsoft/onnxruntime/releases) 참고):

```bash
curl -LO https://github.com/microsoft/onnxruntime/releases/download/v1.20.1/onnxruntime-linux-aarch64-1.20.1.tgz
tar xzf onnxruntime-linux-aarch64-1.20.1.tgz
export ONNXRUNTIME_ROOT=$PWD/onnxruntime-linux-aarch64-1.20.1
```

### 빌드

```bash
cmake -B build -DONNXRUNTIME_ROOT=$ONNXRUNTIME_ROOT
cmake --build build
```

`ONNXRUNTIME_ROOT`는 `-D` 옵션 대신 환경변수로 미리 export해둬도 CMake가
자동으로 인식한다(`export ONNXRUNTIME_ROOT=...` 후 `cmake -B build`만 실행해도 됨).

### 모델 파일

`detection/models/person_detector_script_11_lite.onnx`가 저장소에 포함되어 있다.
CustomDetector 커스텀 grid 모델(11단계 lite)을 ONNX로 export한 것으로,
`PersonDetector`가 기본으로 이 파일을 로드한다.
