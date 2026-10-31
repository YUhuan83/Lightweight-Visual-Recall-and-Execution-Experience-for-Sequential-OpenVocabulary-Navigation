// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <limits>
#include "wildos_ackermann_rpp/curvature.hpp"
using wildos_ackermann_rpp::bounded_curvature;
TEST(Curvature, PreserveSupportedSteering) {
  EXPECT_DOUBLE_EQ(bounded_curvature(.1, 8.), .1);
  EXPECT_DOUBLE_EQ(bounded_curvature(-.1, 8.), -.1);
  EXPECT_DOUBLE_EQ(bounded_curvature(0., 8.), 0.);
}
TEST(Curvature, SameBoundaryForForwardAndReverse) {
  for (double requested : {-.8, -.15, .15, .8}) {
    auto curvature = bounded_curvature(requested, 8.);
    EXPECT_DOUBLE_EQ(std::abs(curvature), .125);
    for (double speed : {-1.2, -.25, .25, 1.2}) {
      EXPECT_DOUBLE_EQ((speed * curvature) / speed, curvature);
    }
  }
}
TEST(Curvature, RejectInvalidContract) {
  EXPECT_THROW(bounded_curvature(.1, 0.), std::invalid_argument);
  EXPECT_THROW(bounded_curvature(.1, -1.), std::invalid_argument);
  EXPECT_THROW(bounded_curvature(std::numeric_limits<double>::quiet_NaN(), 8.), std::invalid_argument);
  EXPECT_THROW(bounded_curvature(.1, std::numeric_limits<double>::infinity()), std::invalid_argument);
}
