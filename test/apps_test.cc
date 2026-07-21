#include <front/apps.hh>

#include <gtest/gtest.h>

#include <cstdio>

namespace {

TEST(Apps, Foo) {
    auto map = front::ThrottleMap::create_default();
    for (std::size_t i = 0; i < 2096; i++) {
        std::printf("%u: %u\n", i, map[i]);
    }
}

} // namespace
