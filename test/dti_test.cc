#include <dti.hh>

#include <gtest/gtest.h>

#include <variant>

namespace {

TEST(DtiParse, GeneralData1) {
    const auto packet = dti::parse_packet(0x2a22, 0x5e240000, 0x86017100);
    ASSERT_TRUE(std::holds_alternative<dti::GeneralData1>(packet));

    const auto gd1 = std::get<dti::GeneralData1>(packet);
    EXPECT_EQ(gd1.erpm, 9310);
    EXPECT_EQ(gd1.duty_cycle, 113);
    EXPECT_EQ(gd1.input_voltage, 390);
}

TEST(DtiParse, GeneralData2) {
    const auto packet = dti::parse_packet(0x2b22, 0x11005c00, 0xffffffff);
    ASSERT_TRUE(std::holds_alternative<dti::GeneralData2>(packet));

    const auto gd2 = std::get<dti::GeneralData2>(packet);
    EXPECT_EQ(gd2.ac_current, 92);
    EXPECT_EQ(gd2.dc_current, 17);
}

TEST(DtiParse, GeneralData3) {
    const auto packet = dti::parse_packet(0x2c22, 0x17015301, 0xffffff00);
    ASSERT_TRUE(std::holds_alternative<dti::GeneralData3>(packet));

    const auto gd3 = std::get<dti::GeneralData3>(packet);
    EXPECT_EQ(gd3.controller_temperature, 339);
    EXPECT_EQ(gd3.motor_temperature, 279);
    EXPECT_EQ(gd3.fault_code, dti::FaultCode::NoFaults);
}

TEST(DtiParse, GeneralData5) {
    const auto packet = dti::parse_packet(0x2e22, 0x01aad838, 0x23ff05aa);
    ASSERT_TRUE(std::holds_alternative<dti::GeneralData5>(packet));

    const auto gd5 = std::get<dti::GeneralData5>(packet);
    EXPECT_EQ(gd5.throttle, 56);
    EXPECT_EQ(gd5.brake, -40);
    EXPECT_EQ(gd5.digital_pin_state, 0xaa);
    EXPECT_TRUE(gd5.drive_enable);
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
}

} // namespace
