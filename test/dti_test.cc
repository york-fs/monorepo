#include <dti.hh>

#include <can.hh>

#include <gtest/gtest.h>

#include <cstdint>
#include <variant>

namespace {

TEST(DtiBuild, SetCurrent) {
    EXPECT_EQ(dti::build_set_current(0x10, 100),
              can::build_extended(0x0110, std::to_array<std::uint8_t>({0x00, 0x64})));
    EXPECT_EQ(dti::build_set_current(0x20, -100),
              can::build_extended(0x0120, std::to_array<std::uint8_t>({0xff, 0x9c})));
}

TEST(DtiBuild, SetBrakeCurrent) {
    EXPECT_EQ(dti::build_set_brake_current(0x10, 100),
              can::build_extended(0x0210, std::to_array<std::uint8_t>({0x00, 0x64})));
}

TEST(DtiBuild, SetErpm) {
    EXPECT_EQ(dti::build_set_erpm(0x10, 500),
              can::build_extended(0x0310, std::to_array<std::uint8_t>({0x00, 0x00, 0x01, 0xf4})));
    EXPECT_EQ(dti::build_set_erpm(0x20, -500),
              can::build_extended(0x0320, std::to_array<std::uint8_t>({0xff, 0xff, 0xfe, 0x0c})));
}

TEST(DtiBuild, SetPosition) {
    EXPECT_EQ(dti::build_set_position(0x22, 1000),
              can::build_extended(0x0422, std::to_array<std::uint8_t>({0x03, 0xe8})));
}

TEST(DtiBuild, SetRelativeCurrent) {
    EXPECT_EQ(dti::build_set_relative_current(0x10, 100),
              can::build_extended(0x0510, std::to_array<std::uint8_t>({0x00, 0x64})));
    EXPECT_EQ(dti::build_set_relative_current(0x20, -100),
              can::build_extended(0x0520, std::to_array<std::uint8_t>({0xff, 0x9c})));
}

TEST(DtiBuild, SetRelativeBrakeCurrent) {
    EXPECT_EQ(dti::build_set_relative_brake_current(0x22, 100),
              can::build_extended(0x0622, std::to_array<std::uint8_t>({0x00, 0x64})));
}

TEST(DtiBuild, SetDriveEnabled) {
    EXPECT_EQ(dti::build_set_drive_enabled(0x10, false),
              can::build_extended(0x0c10, std::to_array<std::uint8_t>({0x00})));
    EXPECT_EQ(dti::build_set_drive_enabled(0x20, true),
              can::build_extended(0x0c20, std::to_array<std::uint8_t>({0x01})));
}

TEST(DtiParse, GeneralData1) {
    const auto message_bytes = std::to_array<std::uint8_t>({0x00, 0x00, 0x24, 0x5e, 0x00, 0x71, 0x01, 0x86});
    const auto frame = can::build_extended(0x2022, message_bytes);
    const auto gd1 = can::decode<dti::GeneralData1>(0x22, frame);
    ASSERT_TRUE(gd1.has_value());
    EXPECT_EQ(gd1->erpm, 9310);
    EXPECT_EQ(gd1->duty_cycle, 113);
    EXPECT_EQ(gd1->input_voltage, 390);
}

TEST(DtiParse, GeneralData2) {
    const auto message_bytes = std::to_array<std::uint8_t>({0x00, 0x5c, 0x00, 0x11});
    const auto frame = can::build_extended(0x2122, message_bytes);
    const auto gd2 = can::decode<dti::GeneralData2>(0x22, frame);
    ASSERT_TRUE(gd2.has_value());
    EXPECT_EQ(gd2->ac_current, 92);
    EXPECT_EQ(gd2->dc_current, 17);
}

TEST(DtiParse, GeneralData3) {
    const auto message_bytes = std::to_array<std::uint8_t>({0x01, 0x53, 0x01, 0x17, 0x04});
    const auto frame = can::build_extended(0x2222, message_bytes);
    const auto gd3 = can::decode<dti::GeneralData3>(0x22, frame);
    ASSERT_TRUE(gd3.has_value());
    EXPECT_EQ(gd3->controller_temperature, 339);
    EXPECT_EQ(gd3->motor_temperature, 279);
    EXPECT_EQ(gd3->fault_code, dti::FaultCode::Overcurrent);
}

TEST(DtiParse, GeneralData5) {
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

} // namespace
