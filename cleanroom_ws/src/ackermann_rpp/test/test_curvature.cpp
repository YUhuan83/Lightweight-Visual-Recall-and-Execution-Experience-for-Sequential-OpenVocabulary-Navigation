#include <gtest/gtest.h>
#include "wildos_ackermann_rpp/curvature.hpp"
TEST(Curvature, BoundsBothDirectionsWithoutChangingSmallCurvature) {
  EXPECT_DOUBLE_EQ(wildos_ackermann_rpp::bounded_curvature(.01,8.),.01);
  EXPECT_DOUBLE_EQ(wildos_ackermann_rpp::bounded_curvature(1.,8.),.125);
  EXPECT_DOUBLE_EQ(wildos_ackermann_rpp::bounded_curvature(-1.,8.),-.125);
  EXPECT_THROW(wildos_ackermann_rpp::bounded_curvature(0.,0.),std::invalid_argument);
}
