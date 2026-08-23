#pragma once

#include <util/numeric.hh>

#include <algorithm>
#include <cstdint>
#include <optional>
#include <span>

namespace util {

class Stream {
    std::span<std::uint8_t> m_span;
    std::size_t m_head{0};

public:
    Stream(std::span<std::uint8_t> span) : m_span(span) {}

    std::size_t read(std::span<std::uint8_t> data);
    std::size_t write(std::span<const std::uint8_t> data);

    std::optional<std::uint8_t> read_byte();
    bool write_byte(std::uint8_t byte);

    template <std::integral T>
    std::optional<T> read_be();
    template <std::integral T>
    bool write_be(T value);

    template <std::integral T>
    std::optional<T> read_le();
    template <std::integral T>
    bool write_le(T value);

    std::span<std::uint8_t> bytes() const { return m_span.subspan(0, m_head); };
    bool empty() const { return m_span.empty(); }
    std::size_t head() const { return m_head; }
};

inline std::size_t Stream::read(std::span<std::uint8_t> data) {
    const auto to_read = std::min(data.size(), m_span.size() - m_head);
    std::copy_n(m_span.begin() + m_head, to_read, data.begin());
    m_head += to_read;
    return to_read;
}

inline std::size_t Stream::write(std::span<const std::uint8_t> data) {
    const auto to_write = std::min(data.size(), m_span.size() - m_head);
    std::copy_n(data.begin(), to_write, m_span.begin() + m_head);
    m_head += to_write;
    return to_write;
}

inline std::optional<std::uint8_t> Stream::read_byte() {
    std::uint8_t byte;
    if (read({&byte, 1}) != 1) {
        return std::nullopt;
    }
    return byte;
}

inline bool Stream::write_byte(std::uint8_t byte) {
    return write({&byte, 1}) == 1;
}

template <std::integral T>
std::optional<T> Stream::read_be() {
    std::array<std::uint8_t, sizeof(T)> bytes;
    if (read(bytes) != sizeof(T)) {
        return std::nullopt;
    }
    return util::read_be<T>(bytes);
}

template <std::integral T>
bool Stream::write_be(T value) {
    const auto bytes = util::write_be<T>(value);
    return write(bytes) == sizeof(T);
}

template <std::integral T>
std::optional<T> Stream::read_le() {
    std::array<std::uint8_t, sizeof(T)> bytes;
    if (read(bytes) != sizeof(T)) {
        return std::nullopt;
    }
    return util::read_le<T>(bytes);
}

template <std::integral T>
bool Stream::write_le(T value) {
    const auto bytes = util::write_le<T>(value);
    return write(bytes) == sizeof(T);
}

} // namespace util
