#pragma once

#include <util/flag_bitset.hh>

#include <cstdint>

namespace rear {

enum class OnlineFlag : std::uint8_t {
    FrontOnline,
    BmsOnline,
    PrechargeOnline,
    InverterOnline,
};

using OnlineFlags = util::FlagBitset<OnlineFlag>;

} // namespace rear
