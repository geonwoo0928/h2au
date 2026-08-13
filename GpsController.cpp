#include "GpsController.h"
#include "SeoilCoordController.h"
#include "RcCarDataManager.h"

GpsController::GpsController(const char* serialPort, RcCarDataManager& dataManager)
    : dataManager_(dataManager) {
    uartFilestream_ = open(serialPort, O_RDONLY | O_NOCTTY);
    if(uartFilestream_ < 0) {
        throw std::runtime_error(std::string("Failed to open ") + serialPort);
    }

    tcflush(uartFilestream_, TCIFLUSH); // 기존에 커널 버퍼에 있던 데이터 비움

    struct termios options;
    tcgetattr(uartFilestream_, &options);

    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    options.c_cflag &= ~PARENB;        // 패리티 비트(Parity) 사용 안 함
    options.c_cflag &= ~CSTOPB;        // 스톱 비트(Stop Bit) 2개 대신 1개 사용
    options.c_cflag &= ~CSIZE;         // 데이터 비트 설정을 초기화(클리어)
    options.c_cflag |= CS8;            // 데이터 비트를 8비트로 설정
    options.c_cflag &= ~CRTSCTS;       // 하드웨어 흐름 제어(RTS/CTS Flow Control) 비활성화
    options.c_cflag |= CREAD | CLOCAL; // 수신기(Receiver) 활성화 및 로컬 라인 제어 (모뎀 제어 신호 무시)

    //    - ICANON 해제: 엔터(\n) 키를 누를 때까지 기다리지 않고, 들어오는 즉시 바이트 단위로 읽음
    //    - ECHO / ECHOE 해제: 입력받은 문장을 터미널에 다시 화면 출력하는 기능 끄기 (수신만 하도록)
    //    - ISIG 해제: Ctrl+C 같은 시그널 문자(인터럽트)를 무시하고 일반 데이터로 처리
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    options.c_oflag &= ~OPOST; // 줄바꿈 문자 등을 시스템 임의로 변환하지 않고 원본 그대로 출력/처리

    tcsetattr(uartFilestream_, TCSANOW, &options);
}

GpsController::~GpsController() {
    if(uartFilestream_ >= 0)
        close(uartFilestream_);
}

void GpsController::runGpsThread(const SeoilCoordController& coordController) {
    while(isThreadRun_) {
        double lat = 0.0;
        double lon = 0.0;

        if(!getGpsData(lat, lon))
            continue;

        cv::Point2f pixel = coordController.getRcCarPixel(lat, lon);
        dataManager_.updateGps(lat, lon, pixel.x, pixel.y);
    }

    return;
}

void GpsController::stopThread() {isThreadRun_ = false;}

bool GpsController::getGpsData(double& lat, double& lon) {
    char buffer[256];

    while(true) {
        size_t pos = rxBuffer_.find('\n');

        if(pos == std::string::npos) {
            int rx_length = read(uartFilestream_, (void*)buffer, sizeof(buffer) - 1);

            if (rx_length <= 0)
                return false;

            rxBuffer_.append(buffer, rx_length);
            
            continue;
        }

        std::string nmeaLine = rxBuffer_.substr(0, pos);
        rxBuffer_.erase(0, pos + 1);
        
        if (!nmeaLine.empty() && nmeaLine.back() == '\r') // gps 센서는 데이터 마지막에 캐리지 리턴 포함해서 반환함
            nmeaLine.pop_back();

        if (nmeaLine.rfind("$GPGGA", 0) != 0) 
            continue; 

        if (parseGpsData(nmeaLine, lat, lon))
            return true;
    }

    return false;
}

bool GpsController::parseGpsData(const std::string& nmeaLine, double& lat, double& lon) {
    std::stringstream stream(nmeaLine);
    std::string token;
    int index = 0;
    std::string lat_str;
    std::string lon_str;

    while (std::getline(stream, token, ',')) {
        if (index == 2) lat_str = token;
        if (index == 4) lon_str = token;
        index++;
    }

    if (lat_str.empty() || lon_str.empty())
    return false;

    if (convertToDegree(lat_str, lat) && convertToDegree(lon_str, lon))
        return true;

    return false;
}

bool GpsController::convertToDegree(const std::string& degreeText, double& degrees) {
    int divisor = 100;
    try {
        double rawDegree = std::stod(degreeText);
        int convertedDegrees = static_cast<int>(rawDegree / divisor);
        double minutes = rawDegree - (convertedDegrees * divisor);
        degrees = convertedDegrees + (minutes / 60.0);
        
        return true;
    } catch (...) {
        return false;
    }
}