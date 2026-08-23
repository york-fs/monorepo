#include <rear/can_messages.hh>

#include <rear/shutdown.hh>
#include <util/stream.hh>

#include <optional>
#include <type_traits>

namespace rear {

std::optional<StatusMessage> StatusMessage::decode(util::Stream &stream) {
    const auto shutdown_open_cause = stream.read_be<std::underlying_type_t<ShutdownCircuitOpenCause>>();
    const auto ts_prevention_flags = stream.read_be<TsPreventionFlags::type_t>();
    const auto rtd_prevention_flags = stream.read_be<RtdPreventionFlags::type_t>();
    if (!shutdown_open_cause || !ts_prevention_flags || !rtd_prevention_flags) {
        return std::nullopt;
    }
    return StatusMessage{
        .shutdown_open_cause = ShutdownCircuitOpenCause(*shutdown_open_cause),
        .ts_prevention_flags = TsPreventionFlags(*ts_prevention_flags),
        .rtd_prevention_flags = RtdPreventionFlags(*rtd_prevention_flags),
    };
}

bool StatusMessage::encode(util::Stream &stream) const {
    if (!stream.write_be(util::to_underlying(shutdown_open_cause))) {
        return false;
    }
    if (!stream.write_be(ts_prevention_flags.value())) {
        return false;
    }
    return stream.write_be(rtd_prevention_flags.value());
}

} // namespace rear
