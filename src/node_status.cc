#include <node_status.hh>

#include <can.hh>
#include <freertos.hh>
#include <util/stream.hh>

#include <cstdint>
#include <optional>

namespace node_status {
namespace {

struct QueueData {
    std::int8_t mcu_temp;
};

freertos::Task<configMINIMAL_STACK_SIZE> s_task;
freertos::Queue<QueueData, 1> s_queue;
std::uint8_t s_node_id = 0;

void tx_task(void *) {
    freertos::PeriodScheduler scheduler;
    while (true) {
        const auto queue_data = *s_queue.receive(portMAX_DELAY);
        const auto can_stats = can::get_stats();

        NodeStatusMessage1 message_1{
            .can_rx_count = can_stats.rx_count,
            .can_tx_count = can_stats.tx_count,
        };
        can::transmit(s_node_id, message_1);

        NodeStatusMessage2 message_2{
            .can_lost_rx_count = can_stats.lost_rx_count,
            .can_lost_tx_count = can_stats.lost_tx_count,
        };
        can::transmit(s_node_id, message_2);

        NodeStatusMessage3 message_3{
            .uptime_ms = freertos::uptime_ms(),
            .mcu_temp = queue_data.mcu_temp,
        };
        can::transmit(s_node_id, message_3);

        scheduler.delay_until_ms(1000);
    }
}

} // namespace

std::optional<NodeStatusMessage1> NodeStatusMessage1::decode(util::Stream &stream) {
    const auto can_rx_count = stream.read_be<std::uint32_t>();
    const auto can_tx_count = stream.read_be<std::uint32_t>();
    if (!can_rx_count || !can_tx_count) {
        return std::nullopt;
    }
    return NodeStatusMessage1{
        .can_rx_count = *can_rx_count,
        .can_tx_count = *can_tx_count,
    };
}

bool NodeStatusMessage1::encode(util::Stream &stream) const {
    if (!stream.write_be(can_rx_count)) {
        return false;
    }
    return stream.write_be(can_tx_count);
}

std::optional<NodeStatusMessage2> NodeStatusMessage2::decode(util::Stream &stream) {
    const auto lost_can_rx_count = stream.read_be<std::uint32_t>();
    const auto lost_can_tx_count = stream.read_be<std::uint32_t>();
    if (!lost_can_rx_count || !lost_can_tx_count) {
        return std::nullopt;
    }
    return NodeStatusMessage2{
        .can_lost_rx_count = *lost_can_rx_count,
        .can_lost_tx_count = *lost_can_tx_count,
    };
}

bool NodeStatusMessage2::encode(util::Stream &stream) const {
    if (!stream.write_be(can_lost_rx_count)) {
        return false;
    }
    return stream.write_be(can_lost_tx_count);
}

std::optional<NodeStatusMessage3> NodeStatusMessage3::decode(util::Stream &stream) {
    const auto uptime_ms = stream.read_be<std::uint32_t>();
    const auto mcu_temp = stream.read_be<std::int8_t>();
    if (!uptime_ms || !mcu_temp) {
        return std::nullopt;
    }
    return NodeStatusMessage3{
        .uptime_ms = *uptime_ms,
        .mcu_temp = *mcu_temp,
    };
}

bool NodeStatusMessage3::encode(util::Stream &stream) const {
    if (!stream.write_be(uptime_ms)) {
        return false;
    }
    return stream.write_be(mcu_temp);
}

void init(std::uint8_t node_id) {
    s_node_id = node_id;
    s_queue.init();
    s_task.init(&tx_task, "node_status", 0);
}

void update(std::uint32_t mcu_temp_voltage) {
    // Calculate an approximate MCU temperature using constants from the datasheet.
    const auto mcu_temp =
        static_cast<std::int8_t>(((1430 - static_cast<std::int32_t>(mcu_temp_voltage)) * 10) / 43 + 25);
    s_queue.overwrite(QueueData{
        .mcu_temp = mcu_temp,
    });
}

} // namespace node_status
