#include <i2c.hh>

#include <hal.hh>
#include <stm32f103xb.h>

#include <cstdint>
#include <utility>

namespace i2c {
namespace {

StateMachine *s_sm1 = nullptr;
StateMachine *s_sm2 = nullptr;

I2C_TypeDef *i2c_for(Bus bus) {
    return bus == Bus::_1 ? I2C1 : I2C2;
}

State handle_event(I2C_TypeDef *i2c, StateMachine *sm) {
    const auto sr1 = i2c->SR1;
    const auto current_state = sm->state();
    if (current_state == State::Start && (sr1 & I2C_SR1_SB) != 0) {
        i2c->DR = sm->address();
        return (sm->address() & 1u) != 0 ? State::AddrRx : State::AddrTx;
    }

    if (current_state == State::AddrRx && (sr1 & I2C_SR1_ADDR) != 0) {
        // Handle edge cases.
        const auto space = sm->remaining_space();
        if (space == 1) {
            i2c->CR1 &= ~I2C_CR1_ACK;
        } else if (space == 2) {
            i2c->CR1 |= I2C_CR1_POS;
        }

        // Clear ADDR bit by reading SR2.
        i2c->SR2;

        // Handle edge cases.
        if (space == 1) {
            i2c->CR1 |= I2C_CR1_STOP;
            return State::Rx;
        }
        if (space == 2 || space == 3) {
            // We only care about BTF events in this case.
            i2c->CR2 &= ~I2C_CR2_ITBUFEN;
            if (space == 2) {
                i2c->CR1 &= ~I2C_CR1_ACK;
            }
            return State::RxFinish;
        }
        return State::Rx;
    }

    if (current_state == State::AddrTx && (sr1 & I2C_SR1_ADDR) != 0) {
        // Clear ADDR bit by reading SR2.
        i2c->SR2;
        return State::Tx;
    }

    // Master receive.
    if (current_state == State::Rx && (sr1 & I2C_SR1_RXNE) != 0) {
        sm->next_byte() = i2c->DR;
        if (sm->remaining_space() == 0) {
            return State::Idle;
        }
        if (sm->remaining_space() == 3) {
            i2c->CR2 &= ~I2C_CR2_ITBUFEN;
            return State::RxFinish;
        }
        return State::Rx;
    }
    if (current_state == State::RxFinish && (sr1 & I2C_SR1_BTF) != 0) {
        const auto space = sm->remaining_space();
        if (space == 3) {
            i2c->CR1 &= ~I2C_CR1_ACK;
        } else if (space == 2) {
            i2c->CR1 |= I2C_CR1_STOP;
            sm->next_byte() = i2c->DR;
        }
        sm->next_byte() = i2c->DR;
        return space == 3 ? State::RxFinish : State::Idle;
    }

    // Master transmit.
    if (current_state == State::Tx && (sr1 & I2C_SR1_TXE) != 0) {
        if (!sm->is_full()) {
            i2c->DR = sm->next_byte();
            return State::Tx;
        }
        // All bytes transmitted. Disable interrupts from TXE setting since we only care about BTF now.
        i2c->CR2 &= ~I2C_CR2_ITBUFEN;
        return State::Stop;
    }

    // Send stop condition once the last byte has been fully transferred.
    if (current_state == State::Stop && (sr1 & I2C_SR1_BTF) != 0) {
        i2c->CR2 &= ~I2C_CR2_ITEVTEN;
        i2c->CR1 |= I2C_CR1_STOP;
        return State::Idle;
    }

    // Unexpected event.
    i2c->CR2 &= ~I2C_CR2_ITEVTEN;
    i2c->CR1 |= I2C_CR1_STOP;
    return State::Error;
}

State handle_error(I2C_TypeDef *i2c) {
    const auto sr1 = i2c->SR1;

    // Clear all error flags.
    i2c->SR1 = 0;

    // Disable event interrupts.
    i2c->CR2 &= ~I2C_CR2_ITEVTEN;

    if ((sr1 & I2C_SR1_AF) != 0) {
        i2c->CR1 |= I2C_CR1_STOP;
        return State::NoAck;
    }
    return State::Error;
}

bool should_notify_state(State state) {
    switch (state) {
    case State::Idle:
    case State::NoAck:
    case State::Error:
        return true;
    default:
        return false;
    }
}

} // namespace

// void StateMachine::init() {
//     I2C_TypeDef *foo;
// }

void StateMachine::reinit() {
    auto *i2c = i2c_for(m_bus);
    i2c->CR1 &= ~I2C_CR1_PE;

    i2c->CR2 |= I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
}

void StateMachine::start(std::uint8_t address, std::span<std::uint8_t> buffer) {
    m_address = address;
    m_buffer = buffer;
    m_head.store(0);
    m_state.store(State::Start);

    // Enable all interrupts and generate a start condition.
    auto *i2c = i2c_for(m_bus);
    i2c->CR2 |= I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
    i2c->CR1 &= ~I2C_CR1_POS;
    i2c->CR1 |= I2C_CR1_START;
}

void StateMachine::start_read(std::uint8_t address, std::span<std::uint8_t> buffer) {
    i2c_for(m_bus)->CR1 |= I2C_CR1_ACK;
    start((address << 1) | 1u, buffer);
}

void StateMachine::start_write(std::uint8_t address, std::span<std::uint8_t> buffer) {
    start(address << 1, buffer);
}

bool StateMachine::event() {
    const auto previous_state = m_state.exchange(handle_event(i2c_for(m_bus), this));
    return previous_state != m_state && should_notify_state(m_state);
}

void StateMachine::error() {
    m_state.store(handle_error(i2c_for(m_bus)));
}

std::uint8_t &StateMachine::next_byte() {
    return m_buffer[m_head++];
}

bool StateMachine::is_full() const {
    return m_head >= m_buffer.size();
}

std::uint32_t StateMachine::remaining_space() const {
    return m_buffer.size() - m_head;
}

} // namespace i2c
