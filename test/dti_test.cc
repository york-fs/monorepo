#include <dti.hh>

#include <can.hh>

#include <gtest/gtest.h>

#include <cstdint>
#include <variant>

namespace {

TEST(DtiParse, GeneralData1) {
    const auto message_bytes = std::to_array<std::uint8_t>({0x00, 0x00, 0x24, 0x5e, 0x00, 0x71, 0x01, 0x86});
    const auto packet = dti::parse_packet(can::build_raw(0x2022, message_bytes));
    ASSERT_TRUE(std::holds_alternative<dti::GeneralData1>(packet));

    const auto gd1 = std::get<dti::GeneralData1>(packet);
    EXPECT_EQ(gd1.erpm, 9310);
    EXPECT_EQ(gd1.duty_cycle, 113);
    EXPECT_EQ(gd1.input_voltage, 390);
}

TEST(DtiParse, GeneralData2) {
    const auto message_bytes = std::to_array<std::uint8_t>({0x00, 0x5c, 0x00, 0x11});
    const auto packet = dti::parse_packet(can::build_raw(0x2122, message_bytes));
    ASSERT_TRUE(std::holds_alternative<dti::GeneralData2>(packet));

    const auto gd2 = std::get<dti::GeneralData2>(packet);
    EXPECT_EQ(gd2.ac_current, 92);
    EXPECT_EQ(gd2.dc_current, 17);
}

TEST(DtiParse, GeneralData3) {
    const auto message_bytes = std::to_array<std::uint8_t>({0x01, 0x53, 0x01, 0x17, 0x00});
    const auto packet = dti::parse_packet(can::build_raw(0x2222, message_bytes));
    ASSERT_TRUE(std::holds_alternative<dti::GeneralData3>(packet));

    const auto gd3 = std::get<dti::GeneralData3>(packet);
    EXPECT_EQ(gd3.controller_temperature, 339);
    EXPECT_EQ(gd3.motor_temperature, 279);
    EXPECT_EQ(gd3.fault_code, dti::FaultCode::NoFaults);
}

TEST(DtiParse, GeneralData5) {
    const auto message_bytes = std::to_array<std::uint8_t>({0x38, 0xd8, 0xaa, 0x01, 0xaa, 0x05, 0xff, 0x18});
    const auto packet = dti::parse_packet(can::build_raw(0x2422, message_bytes));
    ASSERT_TRUE(std::holds_alternative<dti::GeneralData5>(packet));

    const auto gd5 = std::get<dti::GeneralData5>(packet);
    EXPECT_EQ(gd5.throttle, 56);
    EXPECT_EQ(gd5.brake, -40);
    EXPECT_EQ(gd5.digital_pin_state, 0xaa);
    EXPECT_TRUE(gd5.drive_enabled);
    EXPECT_FALSE(gd5.capacitor_temperature_limit_active);
    EXPECT_TRUE(gd5.dc_current_limit_active);
    EXPECT_FALSE(gd5.drive_enable_limit_active);
    EXPECT_TRUE(gd5.igbt_acceleration_limit_active);
    EXPECT_FALSE(gd5.igbt_temperature_limit_active);
    EXPECT_TRUE(gd5.input_voltage_limit_active);
    EXPECT_FALSE(gd5.motor_acceleration_temperature_limit_active);
    EXPECT_TRUE(gd5.motor_temperature_limit_active);
    EXPECT_TRUE(gd5.rpm_min_limit_active);
    EXPECT_FALSE(gd5.rpm_max_limit_active);
    EXPECT_TRUE(gd5.power_limit_active);
    EXPECT_EQ(gd5.can_map_version, 24);
}

TEST(DtiParse, UnknownMessageType) {
    const auto message_bytes = std::to_array<std::uint8_t>({0x12, 0x34});
    const auto packet = dti::parse_packet(can::build_raw(0x4022, message_bytes));
    ASSERT_TRUE(std::holds_alternative<dti::UnknownMessageType>(packet));
    EXPECT_EQ(std::get<dti::UnknownMessageType>(packet).packet_id, 0x40);
}

} // namespace
