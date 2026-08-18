#include "ImuController.h"

#include <chrono>
#include <stdexcept>
#include <thread>

namespace
{
    constexpr uint8_t CHIP_ID_REG = 0x00;     // BNO055 내부의 칩 ID 레지스터 주소
    constexpr uint8_t BNO055_CHIP_ID = 0xA0;  // CHIP_ID_REG에서 읽혀야 하는 BNO055 고유 ID 값
    constexpr uint8_t OPR_MODE_REG = 0x3D;    // BNO055의 동작 모드를 설정하는 레지스터 주소
    constexpr uint8_t SYS_TRIGGER_REG = 0x3F; // 리셋 등 시스템 동작을 설정하는 레지스터 주소
    constexpr uint8_t EULER_H_LSB = 0x1A;     // Euler Heading 데이터가 시작되는 레지스터 주소

    constexpr uint8_t CONFIG_MODE = 0x00; // 설정 변경을 위한 CONFIG 동작 모드 값
    constexpr uint8_t NDOF_MODE = 0x0C;   // 가속도계+자이로+지자기를 융합하는 NDOF 동작 모드 값

    constexpr double EULER_SCALE = 16.0; // 센서 원시값을 각도(°)로 변환하기 위한 스케일 값
}

ImuController::ImuController(I2cDevice &i2c)
    : i2c_(i2c) // 0x28 BNO055 자체를 찾기 위한 I2C 주소
{
}

void ImuController::initialize()
{
    uint8_t chipId = i2c_.readRegister8(CHIP_ID_REG);

    if (chipId != BNO055_CHIP_ID)
        throw std::runtime_error("BNO055 not detected");

    // 설정 변경을 위해 CONFIG mode로 전환
    i2c_.writeRegister8(OPR_MODE_REG, CONFIG_MODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    // 일반 동작 모드
    i2c_.writeRegister8(SYS_TRIGGER_REG, 0x00);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // 가속도 + 자이로 + 지자기 센서 융합
    i2c_.writeRegister8(OPR_MODE_REG, NDOF_MODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
}

int16_t ImuController::combineBytes(uint8_t low, uint8_t high)
{
    uint16_t value =
        static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8);

    return static_cast<int16_t>(value);
}

ImuData ImuController::read()
{
    uint8_t data[6];

    i2c_.readRegisters(EULER_H_LSB, data, sizeof(data));

    int16_t rawHeading = combineBytes(data[0], data[1]);
    int16_t rawRoll = combineBytes(data[2], data[3]);
    int16_t rawPitch = combineBytes(data[4], data[5]);

    ImuData result;

    result.heading = rawHeading / EULER_SCALE;
    result.roll = rawRoll / EULER_SCALE;
    result.pitch = rawPitch / EULER_SCALE;

    return result;
}