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

hardware_limits const relaxed_limits{
    .VCO_LO = 3'500'000'000,
    .VCO_HI = 6'500'000'000,
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

// See https://github.com/mhx/gpsdo-config/issues/2
TEST(Solver, RegressionGithub2) {
  struct {
    uint32_t f1;
    uint32_t f2;
    uint64_t fGPS;
    uint64_t N31;
  } const test_cases[] = {
      {8'765, 4'321, 9'925'486, 5},
      {4'681, 8'729, 5'972'956, 3},
      {4'681, 8'701, 5'972'050, 3},
  };

  GTEST_SKIP();

  for (auto const& tc : test_cases) {
    auto solutions = find_solutions(
        rat64(tc.f1, 1), rat64(tc.f2, 1), relaxed_limits, find::best);
    ASSERT_EQ(solutions.size(), 1);
    auto const& s = solutions.front();
    EXPECT_EQ(s.fGPS, tc.fGPS);
    EXPECT_EQ(s.N31, tc.N31);
    EXPECT_TRUE(
        check_solution(s, relaxed_limits, rat64(tc.f1, 1), rat64(tc.f2, 1)));
  }

  for (auto const& tc : test_cases) {
    auto solutions = find_solutions(
        rat64(tc.f1, 1), rat64(tc.f2, 1), relaxed_limits, find::all);
    ASSERT_GT(solutions.size(), 1);
    auto const& s = solutions.front();
    EXPECT_EQ(s.fGPS, tc.fGPS);
    EXPECT_EQ(s.N31, tc.N31);
    for (auto const& s : solutions) {
      EXPECT_TRUE(
          check_solution(s, relaxed_limits, rat64(tc.f1, 1), rat64(tc.f2, 1)));
    }
  }
}
