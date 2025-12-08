#pragma once

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <type_traits>

namespace util {

template <typename T>
concept enum_concept = std::is_enum_v<T>;

template <typename T>
concept trivially_copyable = std::is_trivially_copyable_v<T>;

template <enum_concept E>
constexpr auto to_underlying(E value) {
    return static_cast<std::underlying_type_t<E>>(value);
}

template <enum_concept T>
class FlagBitset {
    using type_t = std::underlying_type_t<T>;
    static_assert(std::is_integral_v<type_t> && std::is_unsigned_v<type_t>);

private:
    type_t m_value;

    constexpr FlagBitset(type_t value) : m_value(value) {}
    constexpr type_t flag_bit(T flag) const { return type_t(type_t(1) << util::to_underlying(flag)); }

public:
    constexpr FlagBitset() : m_value(0) {}

    template <typename... U>
    constexpr FlagBitset(U... flag)
        requires(std::is_same_v<T, U> && ...)
        : m_value((flag_bit(flag) | ...)) {}

    constexpr operator type_t() const { return m_value; }
    constexpr void clear() { m_value = 0; }
    constexpr void set(T flag) { m_value |= flag_bit(flag); }
    constexpr void set_all(FlagBitset other) { m_value |= other.m_value; }
    constexpr void unset(T flag) { m_value &= ~flag_bit(flag); }
    constexpr void unset_all(FlagBitset other) { m_value &= ~other.m_value; }
    constexpr bool is_set(T flag) const { return (m_value & flag_bit(flag)) != 0; }
    constexpr bool any_set() const { return m_value != 0; }
    constexpr type_t value() const { return m_value; }
};

template <typename Callback>
class ScopeGuard {
    const Callback m_callback;

public:
    ScopeGuard(Callback &&callback) : m_callback(std::move(callback)) {}
    ScopeGuard(const ScopeGuard &) = delete;
    ScopeGuard(ScopeGuard &&) = delete;
    ~ScopeGuard() { m_callback(); }

    ScopeGuard &operator=(const ScopeGuard &) = delete;
    ScopeGuard &operator=(ScopeGuard &&) = delete;
};

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

    bool empty() const { return m_span.empty(); }
};

/**
 * Clamps the given value to the range [min_value, max_value].
 *
 * @param value the value to clamp
 * @param min_value the lower boundary to clamp to
 * @param max_value the upper boundary to clamp to
 * @return either value, min_value, or max_value
 */
template <std::integral T>
constexpr T clamp(T value, std::type_identity_t<T> min_value, std::type_identity_t<T> max_value) {
    return value < min_value ? min_value : (max_value < value ? max_value : value);
}

/**
 * Converts big endian bytes into a signed or unsigned integral.
 *
 * @param bytes a correctly sized span of big endian bytes
 * @return the constructed integral
 */
template <std::integral T>
constexpr T read_be(std::span<const std::uint8_t, sizeof(T)> bytes) {
    std::make_unsigned_t<T> value = 0;
    for (std::size_t i = 0; i < sizeof(T); i++) {
        const auto shift = (sizeof(T) - i - 1) * T(8);
        value |= static_cast<T>(bytes[i]) << shift;
    }
    return static_cast<T>(value);
}

/**
 * Converts a signed or unsigned integral into big endian bytes.
 *
 * @param value the value to convert
 * @return an array of big endian bytes
 */
template <std::integral T>
constexpr std::array<std::uint8_t, sizeof(T)> write_be(T value) {
    std::array<std::uint8_t, sizeof(T)> bytes;
    for (std::size_t i = 0; i < sizeof(T); i++) {
        const auto shift = (sizeof(T) - i - 1) * T(8);
        bytes[i] = static_cast<std::uint8_t>((static_cast<std::make_unsigned_t<T>>(value) >> shift) & 0xffu);
    }
    return bytes;
}

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

} // namespace util
