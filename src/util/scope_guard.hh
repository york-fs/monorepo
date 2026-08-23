#pragma once

#include <utility>

namespace util {

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

} // namespace util
