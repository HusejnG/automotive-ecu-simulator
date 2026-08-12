// test_placeholder.cpp
//
// Placeholder test so the CI pipeline and CMake test target are wired up
// from day one. Real tests get added alongside each module as it's built
// (can_bus, ecu_nodes, diagnostics, fault_injection).

#include <gtest/gtest.h>

TEST(Scaffold, BuildPipelineIsWired) {
    EXPECT_TRUE(true);
}
