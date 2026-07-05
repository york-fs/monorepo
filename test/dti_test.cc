#include <dti.hh>

#include <can.hh>

#include <gtest/gtest.h>

#include <cstdint>
#include <variant>

namespace {

TEST(DtiDecode, GeneralData1) {
    const auto message_bytes = std::to_array<std::uint8_t>({0x00, 0x00, 0x24, 0x5e, 0x00, 0x71, 0x01, 0x86});
    const auto frame = can::build_extended(0x2022, message_bytes);
    const auto gd1 = can::decode<dti::GeneralData1>(0x22, frame);
    ASSERT_TRUE(gd1.has_value());
    EXPECT_EQ(gd1->erpm, 9310);
    EXPECT_EQ(gd1->duty_cycle, 113);
    EXPECT_EQ(gd1->input_voltage, 390);
}

TEST(DtiDecode, GeneralData2) {
    const auto message_bytes = std::to_array<std::uint8_t>({0x00, 0x5c, 0x00, 0x11});
    const auto frame = can::build_extended(0x2122, message_bytes);
    const auto gd2 = can::decode<dti::GeneralData2>(0x22, frame);
    ASSERT_TRUE(gd2.has_value());
    EXPECT_EQ(gd2->ac_current, 92);
    EXPECT_EQ(gd2->dc_current, 17);
}

TEST(DtiDecode, GeneralData3) {
    const auto message_bytes = std::to_array<std::uint8_t>({0x01, 0x53, 0x01, 0x17, 0x04});
    const auto frame = can::build_extended(0x2222, message_bytes);
    const auto gd3 = can::decode<dti::GeneralData3>(0x22, frame);
    ASSERT_TRUE(gd3.has_value());
    EXPECT_EQ(gd3->controller_temperature, 339);
    EXPECT_EQ(gd3->motor_temperature, 279);
    EXPECT_EQ(gd3->fault_code, dti::FaultCode::Overcurrent);
}

TEST(DtiDecode, GeneralData5) {
    const auto message_bytes = std::to_array<std::uint8_t>({0x38, 0xd8, 0xaa, 0x01, 0xaa, 0x05, 0xff, 0x18});
    const auto frame = can::build_extended(0x2422, message_bytes);
    const auto gd5 = can::decode<dti::GeneralData5>(0x22, frame);
    ASSERT_TRUE(gd5.has_value());
    EXPECT_EQ(gd5->throttle, 56);
    EXPECT_EQ(gd5->brake, -40);
    EXPECT_EQ(gd5->digital_pin_state, 0xaa);
    EXPECT_TRUE(gd5->drive_enabled);
    EXPECT_FALSE(gd5->limit_flags.is_set(dti::Limit::CapacitorTemperature));
    EXPECT_TRUE(gd5->limit_flags.is_set(dti::Limit::DcCurrent));
    EXPECT_FALSE(gd5->limit_flags.is_set(dti::Limit::DriveEnable));
    EXPECT_TRUE(gd5->limit_flags.is_set(dti::Limit::IgbtAcceleration));
    EXPECT_FALSE(gd5->limit_flags.is_set(dti::Limit::IgbtTemperature));
    EXPECT_TRUE(gd5->limit_flags.is_set(dti::Limit::InputVoltage));
    EXPECT_FALSE(gd5->limit_flags.is_set(dti::Limit::MotorAccelerationTemperature));
    EXPECT_TRUE(gd5->limit_flags.is_set(dti::Limit::MotorTemperature));
    EXPECT_TRUE(gd5->limit_flags.is_set(dti::Limit::RpmMinimum));
    EXPECT_FALSE(gd5->limit_flags.is_set(dti::Limit::RpmMaximum));
    EXPECT_TRUE(gd5->limit_flags.is_set(dti::Limit::PowerLimit));
    EXPECT_EQ(gd5->can_map_version, 24);
}

TEST(DtiDecode, GeneralData5_BadLength) {
    const auto message_bytes = std::to_array<std::uint8_t>({0x38, 0xd8, 0xaa, 0x01, 0xaa, 0x05, 0x18});
    const auto frame = can::build_extended(0x2422, message_bytes);
    EXPECT_FALSE(can::decode<dti::GeneralData5>(0x22, frame));
}

TEST(DtiDecode, UnknownMessageType) {
    const auto message_bytes = std::to_array<std::uint8_t>({0x12, 0x34});
    const auto frame = can::build_extended(0x4022, message_bytes);
    EXPECT_FALSE(can::decode<dti::GeneralData1>(0x22, frame));
    EXPECT_FALSE(can::decode<dti::GeneralData2>(0x22, frame));
    EXPECT_FALSE(can::decode<dti::GeneralData3>(0x22, frame));
    EXPECT_FALSE(can::decode<dti::GeneralData5>(0x22, frame));
}

} // namespace
