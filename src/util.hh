#pragma once

#include <concepts>
#include <cstdint>
#include <type_traits>

namespace util {

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

} // namespace util
