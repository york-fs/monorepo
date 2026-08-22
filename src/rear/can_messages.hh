#pragma once

#include <rear/shutdown.hh>
#include <util.hh>

#include <cstdint>

namespace rear {

struct StatusMessage {
    ShutdownCircuitOpenCause shutdown_open_cause;
    TsPreventionFlags ts_prevention_flags;
    RtdPreventionFlags rtd_prevention_flags;

    static constexpr std::uint32_t packet_id() { return 0x200; }
    static constexpr std::uint32_t default_priority() { return 1; }
    static std::optional<StatusMessage> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

} // namespace rear
