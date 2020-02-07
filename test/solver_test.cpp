#include "../solver.h"
#include <gtest/gtest.h>

using namespace gpsdo_config;

namespace {

hardware_limits limits{
    .VCO_LO = 4'850'000'000,
    .VCO_HI = 5'670'000'000,
    .F3_LO = 2'000,
    .F3_HI = 2'000'000,
    .GPS_HI = 10'000'000,
};

}

TEST(Solver, BasicTest) {
  auto solutions
      = find_solutions(rat64(123431, 100), rat64(5432, 1), limits, find::all);
  ASSERT_EQ(solutions.size(), 16);
  auto const& s = solutions.front();
  auto f3_got = rat64{s.fGPS, s.N31};
  auto f3_exp = rat64(1974896, 1);
  EXPECT_EQ(f3_got.numerator(), f3_exp.numerator());
  EXPECT_EQ(f3_got.denominator(), f3_exp.denominator());
}
