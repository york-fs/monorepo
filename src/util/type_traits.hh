#pragma once

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

} // namespace util
