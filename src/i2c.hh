#pragma once

#include <atomic>
#include <cstdint>
#include <span>

namespace i2c {

enum class Bus {
    _1,
    _2,
};

enum class State {
    Idle,
    Start,
    AddrRx,
    AddrTx,
    Rx,
    RxFinish,
    Tx,
    Stop,
    NoAck,
    Error,
};

class StateMachine {
    std::span<std::uint8_t> m_buffer;
    std::atomic<std::uint32_t> m_head;
    std::atomic<State> m_state;
    Bus m_bus;
    std::uint8_t m_address{0};

    void start(std::uint8_t address, std::span<std::uint8_t> buffer);

public:
    explicit StateMachine(Bus bus) : m_bus(bus) {}

    void init();

    /**
     * @brief Reinitialise peripheral in case of bus lockup or timeout.
     */
    void reinit();

    void start_read(std::uint8_t address, std::span<std::uint8_t> buffer);
    void start_write(std::uint8_t address, std::span<std::uint8_t> buffer);

    bool event();
    void error();

    std::uint8_t &next_byte();

    bool is_full() const;
    std::uint32_t remaining_space() const;
    State state() const { return m_state.load(); }
    std::uint8_t address() const { return m_address; }
};

} // namespace i2c
