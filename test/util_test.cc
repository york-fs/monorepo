#include <util.hh>

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <span>

namespace {

TEST(Util, Clamp) {
    EXPECT_EQ(util::clamp(1000, -1000, 2000), 1000);
    EXPECT_EQ(util::clamp(-50, 0, 1000), 0);
    EXPECT_EQ(util::clamp(10000, -2000, 2000), 2000);
}

TEST(Util, ReadBe) {
    EXPECT_EQ(util::read_be<std::uint8_t>(std::to_array<std::uint8_t>({0xff})), 0xff);
    EXPECT_EQ(util::read_be<std::uint16_t>(std::to_array<std::uint8_t>({0x42, 0x68})), 17000);
    EXPECT_EQ(util::read_be<std::uint32_t>(std::to_array<std::uint8_t>({0x00, 0x00, 0x42, 0x68})), 17000);
    EXPECT_EQ(util::read_be<std::uint32_t>(std::to_array<std::uint8_t>({0x02, 0x8c, 0xae, 0xda})), 42774234);
    EXPECT_EQ(util::read_be<std::int32_t>(std::to_array<std::uint8_t>({0xff, 0xff, 0x03, 0x79})), -64647);
}

TEST(Util, WriteBe) {
    EXPECT_EQ(util::write_be<std::uint8_t>(0xff), std::to_array<std::uint8_t>({0xff}));
    EXPECT_EQ(util::write_be<std::uint16_t>(51929), std::to_array<std::uint8_t>({0xca, 0xd9}));
    EXPECT_EQ(util::write_be<std::uint32_t>(48934821), std::to_array<std::uint8_t>({0x02, 0xea, 0xaf, 0xa5}));
    EXPECT_EQ(util::write_be<std::int32_t>(-484839), std::to_array<std::uint8_t>({0xff, 0xf8, 0x9a, 0x19}));
}

} // namespace
