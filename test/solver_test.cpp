#include "../solver.h"
#include <gtest/gtest.h>

using namespace gpsdo_config;

namespace {

hardware_limits const limits{
    .VCO_LO = 4'850'000'000,
    .VCO_HI = 5'670'000'000,
    .F3_LO = 2'000,
    .F3_HI = 2'000'000,
    .GPS_HI = 10'000'000,
};

bool check_solution(
    solution const& sol,
    hardware_limits const& lim,
    rat64 f1_exp,
    rat64 f2_exp) {
  auto const f3 = rat64{sol.fGPS, sol.N31};
  auto const fOSC = f3 * sol.N2_HS * sol.N2_LS;
  auto const f1_got = fOSC / (sol.N1_HS * sol.NC1_LS);
  auto const f2_got = fOSC / (sol.N1_HS * sol.NC2_LS);
  bool result = true;

  if (f3 < lim.F3_LO || f3 > lim.F3_HI) {
    std::cerr << "f3 out of range: " << f3 << " [" << lim.F3_LO << " .. "
              << lim.F3_HI << "]\n";
    result = false;
  }

  if (fOSC < lim.VCO_LO || fOSC > lim.VCO_HI) {
    std::cerr << "fOSC out of range: " << fOSC << " [" << lim.VCO_LO << " .. "
              << lim.VCO_HI << "]\n";
    result = false;
  }

  if (f1_got != f1_exp) {
    std::cerr << "f1 mismatch: expected " << f1_exp << ", got " << f1_got
              << "\n";
    result = false;
  }

  if (f2_got != f2_exp) {
    std::cerr << "f2 mismatch: expected " << f2_exp << ", got " << f2_got
              << "\n";
    result = false;
  }

  return result;
}

} // namespace

TEST(Solver, BasicTest) {
  auto solutions
      = find_solutions(rat64(123'431, 100), rat64(5'432, 1), limits, find::all);
  ASSERT_EQ(solutions.size(), 16);
  auto const& s = solutions.front();
  auto f3_got = rat64{s.fGPS, s.N31};
  auto f3_exp = rat64(1'974'896, 1);
  EXPECT_EQ(f3_got.numerator(), f3_exp.numerator());
  EXPECT_EQ(f3_got.denominator(), f3_exp.denominator());

  for (auto const& s : solutions) {
    EXPECT_TRUE(
        check_solution(s, limits, rat64(123'431, 100), rat64(5'432, 1)));
  }
}
