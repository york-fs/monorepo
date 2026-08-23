#pragma once

#include <concepts>
#include <cstdint>
#include <span>
#include <type_traits>

namespace util {

/**
 * @brief Reverses the order of bits in an unsigned integer.
 *
 * @param value the value to reverse
 * @return bit-reversed value
 */
template <std::unsigned_integral T>
constexpr T bit_reverse(T value) {
    T reversed = 0;
    for (std::size_t i = 0; i < sizeof(T) * 8; i++) {
        reversed = (reversed << 1) | ((value >> i) & T(1));
    }
    return reversed;
}

/**
 * @brief Clamps the given value to the range [min_value, max_value].
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
 * @brief Converts big endian bytes into a signed or unsigned integral.
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
 * @brief Converts little endian bytes into a signed or unsigned integral.
 *
 * @param bytes a correctly sized span of little endian bytes
 * @return the constructed integral
 */
template <std::integral T>
constexpr T read_le(std::span<const std::uint8_t, sizeof(T)> bytes) {
    std::make_unsigned_t<T> value = 0;
    for (std::size_t i = 0; i < sizeof(T); i++) {
        value |= static_cast<T>(bytes[i]) << (i * T(8));
    }
    return static_cast<T>(value);
}

/**
 * @brief Converts a signed or unsigned integral into big endian bytes.
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

/**
 * @brief Converts a signed or unsigned integral into little endian bytes.
 *
 * @param value the value to convert
 * @return an array of little endian bytes
 */
template <std::integral T>
constexpr std::array<std::uint8_t, sizeof(T)> write_le(T value) {
    std::array<std::uint8_t, sizeof(T)> bytes;
    for (std::size_t i = 0; i < sizeof(T); i++) {
        const auto shift = i * T(8);
        bytes[i] = static_cast<std::uint8_t>((static_cast<std::make_unsigned_t<T>>(value) >> shift) & 0xffu);
    }
    return bytes;
}

} // namespace util
