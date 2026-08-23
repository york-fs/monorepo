#pragma once

#include <util/type_traits.hh>

#include <bit>
#include <concepts>
#include <type_traits>

namespace util {

template <enum_concept T>
class FlagBitset {
public:
    using type_t = std::underlying_type_t<T>;
    static_assert(std::is_integral_v<type_t> && std::is_unsigned_v<type_t>);

private:
    type_t m_value;

    constexpr type_t flag_bit(T flag) const { return type_t(type_t(1) << util::to_underlying(flag)); }

public:
    constexpr FlagBitset() : m_value(0) {}
    constexpr explicit FlagBitset(type_t value) : m_value(value) {}

    template <std::same_as<T>... U>
    constexpr explicit FlagBitset(U... flag) : m_value((flag_bit(flag) | ...)) {}

    constexpr void clear(T flag) { m_value &= ~flag_bit(flag); }
    constexpr void clear_all(FlagBitset other) { m_value &= ~other.m_value; }
    constexpr void clear_all() { m_value = 0; }

    constexpr void set(T flag) { m_value |= flag_bit(flag); }
    constexpr void set_all(FlagBitset other) { m_value |= other.m_value; }

    constexpr bool is_clear(T flag) const { return (m_value & flag_bit(flag)) == 0; }
    constexpr bool is_set(T flag) const { return !is_clear(flag); }

    constexpr bool any_set() const { return m_value != 0; }
    constexpr bool none_set() const { return m_value == 0; }
    constexpr bool only_set(T flag) const { return m_value == flag_bit(flag); }

    constexpr type_t set_count() const { return type_t(std::popcount(m_value)); }
    constexpr type_t value() const { return m_value; }
};

} // namespace util
