#include <i2c.hh>

#include <hal.hh>
#include <stm32f103xb.h>

#include <cstdint>

namespace i2c {
namespace {

I2C_TypeDef *i2c_for(Bus bus) {
    return bus == Bus::_1 ? I2C1 : I2C2;
}

State handle_event(I2C_TypeDef *i2c, StateMachine *sm) {
    const auto sr1 = i2c->SR1;
    const auto state = sm->state();

    // Master start condition.
    if (state == State::Start && (sr1 & I2C_SR1_SB) != 0) {
        i2c->DR = sm->address();
        return (sm->address() & 1u) != 0 ? State::AddrRx : State::AddrTx;
    }

    // Slave has acknowledged address.
    if (state == State::AddrRx && (sr1 & I2C_SR1_ADDR) != 0) {
        // Handle edge cases. From the datasheet:
        //  1 byte - clear ACK, clear ADDR, program STOP (or START), read the single byte after RXNE
        //  2 bytes - set POS and ACK, clear ADDR, clear ACK, program stop after BTF (handled in RxFinish)
        //  3 bytes - clear ADDR, wait for BTF, clear ACK, read one byte
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
    if (state == State::AddrTx && (sr1 & I2C_SR1_ADDR) != 0) {
        // Clear ADDR bit by reading SR2.
        i2c->SR2;
        return State::Tx;
    }

    // Slave address match.
    if (state == State::Idle && (sr1 & I2C_SR1_ADDR) != 0) {
        // Clear ADDR bit by reading SR2.
        const auto sr2 = i2c->SR2;
        return (sr2 & I2C_SR2_TRA) == 0 ? State::SlaveRx : State::SlaveTx;
    }

    // Master receive.
    if (state == State::Rx && (sr1 & I2C_SR1_RXNE) != 0) {
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
    if (state == State::RxFinish && (sr1 & I2C_SR1_BTF) != 0) {
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
    if (state == State::Tx && (sr1 & I2C_SR1_TXE) != 0) {
        if (!sm->is_at_end()) {
            i2c->DR = sm->next_byte();
            return State::Tx;
        }
        // All bytes transmitted. Disable interrupts from TXE setting since we only care about BTF now.
        i2c->CR2 &= ~I2C_CR2_ITBUFEN;
        return State::Stop;
    }

    // Send stop condition once the last byte has been fully transferred.
    if (state == State::Stop && (sr1 & I2C_SR1_BTF) != 0) {
        i2c->CR2 &= ~I2C_CR2_ITEVTEN;
        i2c->CR1 |= I2C_CR1_STOP;
        return State::Idle;
    }

    // Slave receive.
    if (state == State::SlaveRx && (sr1 & I2C_SR1_RXNE) != 0) {
        const auto byte = i2c->DR;
        if (!sm->is_at_end()) {
            sm->next_byte() = byte;
        }
        if (sm->is_at_end()) {
            i2c->CR1 &= ~I2C_CR1_ACK;
        }
        return State::SlaveRx;
    }

    // Slave transmit.
    if (state == State::SlaveTx && (sr1 & I2C_SR1_TXE) != 0) {
        i2c->DR = !sm->is_at_end() ? sm->next_byte() : 0xff;
        return State::SlaveTx;
    }

    // Slave stop.
    if ((state == State::SlaveRx || state == State::SlaveTx) && (sr1 & I2C_SR1_STOPF) != 0) {
        // Clear the stop flag by writing to CR1.
        i2c->CR1 |= 0;
        return state == State::SlaveRx ? State::SlaveRxFinish : State::Idle;
    }

    // Unexpected event. Disable further event interrupts to avoid a flood.
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

} // namespace

void StateMachine::init() {
    auto *i2c = i2c_for(m_bus);

    // Make sure the peripheral clock is active.
    RCC->APB1ENR |= (i2c == I2C1 ? RCC_APB1ENR_I2C1EN : RCC_APB1ENR_I2C2EN);

    // Disable the peripheral.
    i2c->CR1 &= ~I2C_CR1_PE;

    // Reset the peripheral. This is more aggressive than setting SWRST but seems to handle lockups better, even when
    // following the errata sheet guidance.
    RCC->APB1RSTR = (i2c == I2C1 ? RCC_APB1RSTR_I2C1RST : RCC_APB1RSTR_I2C2RST);
    RCC->APB1RSTR = 0u;

    // Configure peripheral clock frequency.
    i2c->CR2 = ::hal_low_power() ? 8u : 28u;

    // Configure for 100 kHz.
    // TODO: Calculate this properly and allow different speeds.
    if (hal_low_power()) {
        i2c->CCR = 40u;
        i2c->TRISE = 9u;
    } else {
        i2c->CCR = 140u;
        i2c->TRISE = 29u;
    }

    // Enable the peripheral.
    i2c->CR1 |= I2C_CR1_PE;
    m_state.store(State::Idle);
}

void StateMachine::listen(std::uint8_t address, bool engc) {
    auto *i2c = i2c_for(m_bus);
    i2c->OAR1 = address << 1;
    if (engc) {
        i2c->CR1 |= I2C_CR1_ENGC;
    } else {
        i2c->CR1 &= ~I2C_CR1_ENGC;
    }
    m_state.store(State::Idle);

    // Enable all interrupts and own address acknowledgement.
    i2c->CR2 |= I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
    i2c->CR1 |= I2C_CR1_ACK;
}

void StateMachine::set_buffer(std::span<std::uint8_t> buffer) {
    m_buffer = buffer;
    m_head.store(0);
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
    return previous_state != m_state;
}

void StateMachine::error() {
    m_state.store(handle_error(i2c_for(m_bus)));
}

std::uint8_t &StateMachine::next_byte() {
    return m_buffer[m_head++];
}

bool StateMachine::is_at_end() const {
    return m_head >= m_buffer.size();
}

std::uint32_t StateMachine::remaining_space() const {
    return m_buffer.size() - m_head;
}

} // namespace i2c
