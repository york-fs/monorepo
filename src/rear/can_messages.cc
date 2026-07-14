#include <rear/can_messages.hh>

#include <util.hh>

#include <optional>

namespace rear {

std::optional<FlashMessage> FlashMessage::decode(util::Stream &) {
    return FlashMessage{};
}

bool FlashMessage::encode(util::Stream &) const {
    return true;
}

} // namespace rear
