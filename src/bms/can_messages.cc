#include <bms/can_messages.hh>

#include <util.hh>

#include <cstdint>

namespace bms {

bool MasterStatusMessage::encode(util::Stream &stream) {
    if (!stream.write_be(m_error_flags.value())) {
        return false;
    }
    if (!stream.write_byte(m_mcu_temperature)) {
        return false;
    }
    return stream.write_byte(m_shutdown_activated ? 1 : 0);
}

} // namespace bms
