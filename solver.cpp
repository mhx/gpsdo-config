/*
 * GPSDO Configuration Library
 *
 * Copyright (c) 2020 Marcus Holland-Moritz (github@mhxnet.de)
 *
 * This file is part of gpsdo-config.
 *
 * gpsdo-config is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * gpsdo-config is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with gpsdo-config.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "solver.h"

#include <algorithm>
#include <cassert>
#include <numeric>
#include <optional>
#include <unordered_set>

#include <boost/functional/hash.hpp>
#include <boost/integer/common_factor.hpp>
#include <boost/numeric/conversion/cast.hpp>

namespace std {
template <>
struct hash<gpsdo_config::rat64> {
  std::size_t operator()(gpsdo_config::rat64 const& r) const noexcept {
    std::size_t seed = 0;
    boost::hash_combine(seed, r.numerator());
    boost::hash_combine(seed, r.denominator());
    return seed;
  }
};
} // namespace std

namespace gpsdo_config {

namespace {

/**
 * Find the least common multiple of two rational numbers
 */
rat64 rat_lcm(rat64 r1, rat64 r2) {
  int64_t den = boost::integer::lcm(r1.denominator(), r2.denominator());
  int64_t num = boost::integer::lcm(
      r1.numerator() * (den / r1.denominator()),
      r2.numerator() * (den / r2.denominator()));
  return rat64(num, den);
}

#ifndef NDEBUG
bool is_in_ncx_ls_range(int64_t n) {
  // n = 1 should be supported according to the Silicon Labs Si53xx
  // documentation, but writing a value 1 to the GPS reference clock
  // doesn't work as intended.
  //
  // Turns out that it's an undocumented "feature" that n = 1 isn't
  // supported in CMOS mode.
  // (See https://github.com/simontheu/lb-gps-linux/issues/4)
  return /* n == 1 or */ (n <= (1 << 20) and n % 2 == 0);
}
#endif

std::vector<int32_t> factorize(int64_t n) {
  std::vector<int32_t> factors;

  while (n % 2 == 0) {
    factors.emplace_back(2);
    n /= 2;
  }

  for (int i = 3; i <= std::sqrt(n); i += 2) {
    while (n % i == 0) {
      factors.emplace_back(i);
      n /= i;
    }
  }

  if (n > 2) {
    factors.emplace_back(n);
  }

  return factors;
}

int64_t split_rec(
    std::unordered_set<int64_t>& seen,
    int64_t product,
    int64_t limit,
    std::vector<int32_t> const& factors,
    unsigned index) {
  int64_t rv = 1;

  if (seen.insert(product).second) {
    while (index < factors.size()) {
      auto current = factors[index];
      auto res = product / current;

      if (res <= limit) {
        if (res > rv) {
          rv = res;
          break;
        }
      }

      if (index + 1 < factors.size()) {
        auto rr = split_rec(seen, res, limit, factors, index + 1);

        if (rr > rv) {
          rv = rr;
        }
      }

      do {
        ++index;
      } while (index < factors.size() and factors[index] == current);
    }
  }

  return rv;
}

int64_t largest_factor(int64_t product, int64_t limit) {
  std::unordered_set<int64_t> seen;
  return split_rec(seen, product, limit, factorize(product), 0);
}

} // namespace

void solution::write(std::ostream& os, bool verbose) const {
  os << "fGPS = " << fGPS << ", N31 = " << N31 << ", N1_HS = " << N1_HS
     << ", NC1_LS = " << NC1_LS << ", NC2_LS = " << NC2_LS
     << ", N2_HS = " << N2_HS << ", N2_LS = " << N2_LS;

  if (verbose) {
    auto f3 = rat64{fGPS, N31};
    auto fOSC = f3 * N2_HS * N2_LS;
    auto f1 = fOSC / (N1_HS * NC1_LS);
    auto f2 = fOSC / (N1_HS * NC2_LS);
    os << " [f3 = " << boost::rational_cast<double>(f3)
       << ", fOSC = " << boost::rational_cast<double>(fOSC)
       << ", f1 = " << boost::rational_cast<double>(f1)
       << ", f2 = " << boost::rational_cast<double>(f2) << "]";
  }
}

// This allows sorting solutions in order of decreasing PLL frequency f3.
bool solution::operator<(solution const& rhs) const {
  return static_cast<double>(fGPS) / N31
         > static_cast<double>(rhs.fGPS) / rhs.N31;
}

std::vector<solution> find_solutions(
    rat64 f1, rat64 f2, hardware_limits const& limits, find algorithm) {
  int64_t constexpr NCn_LS_MAX = 1 << 20;
  int64_t constexpr N2_LS_MAX = 1 << 20;
  int64_t constexpr N3_MAX = 1 << 19;

  //
  // We need to find a configuration for the Si53xx that can represent both
  // frequencies f1 and f2. The frequencies generated are defined as follows:
  //
  //            fOSC             |  N1_HS  = [4, 5, ..., 11]
  //   fn = --------------       |  NCn_LS = [2, 4, 6, ..., 2**20]
  //        N1_HS * NCn_LS
  //
  // So we first need to find the least common multiple of both frequencies.
  //
  auto fLCM = rat_lcm(f1, f2);

  //
  // As NCn_LS must be even, we check if the LCM divides any of the frequencies
  // into an odd number and double the LCM if necessary.
  //
  if (boost::rational_cast<int64_t>(fLCM / f1) % 2 != 0
      or boost::rational_cast<int64_t>(fLCM / f2) % 2 != 0) {
    fLCM *= 2;
  }

  //
  // We can now compute the base divisors for both frequencies. NC1_LS and
  // NC2_LS must be integer multiples of these divisors:
  //
  //   NCn_LS = q * fn_div
  //
  auto const f1_div = boost::rational_cast<int64_t>(fLCM / f1);
  auto const f2_div = boost::rational_cast<int64_t>(fLCM / f2);

  // Compute the maximum possible value of q to limit our search space.
  auto const q_max = NCn_LS_MAX / std::max(f1_div, f2_div);

  std::unordered_set<rat64> fOSC_seen;
  std::vector<solution> solutions;

  std::optional<find> found;

  for (uint32_t N1_HS = 11; N1_HS >= 4; --N1_HS) {
    //
    // We need a mulitple of fLCM at the output of the high speed divider N1_HS.
    // fN1 is just fLCM before division by N1_HS.
    //
    //               fOSC
    //   fn = ------------------  ,  fLCM = fn * fn_div
    //        N1_HS * q * fn_div
    //
    //            fOSC
    //   fLCM = ---------
    //          N1_HS * q
    //
    //                        fOSC
    //   fN1 = fLCM * N1_HS = ----
    //                         q
    //
    auto const fN1 = N1_HS * fLCM;

    // From the limits of the VCO imposed on fOSC, we can dervice bounds for q.
    auto const q_lo = static_cast<int64_t>(
        std::ceil(boost::rational_cast<double>(limits.VCO_LO / fN1)));
    auto const q_hi = std::min(
        q_max, static_cast<int64_t>(std::floor(
                   boost::rational_cast<double>(limits.VCO_HI / fN1))));

    for (uint32_t q = q_lo; q <= q_hi; ++q) {
      int64_t const NC1_LS = q * f1_div;
      int64_t const NC2_LS = q * f2_div;

      assert(is_in_ncx_ls_range(NC1_LS));
      assert(is_in_ncx_ls_range(NC2_LS));

      auto fOSC = fLCM * q * N1_HS;

      if (fOSC_seen.insert(fOSC).second) {
        // Generate a list of candidates for N2_HS. We start by filling the list
        // with all allowed value in reverse order (the reason being that larger
        // values for the high-speed divider result in lower power consumption).
        std::array<uint32_t, 8> N2_HS_candidates;
        std::iota(N2_HS_candidates.rbegin(), N2_HS_candidates.rend(), 4);

        // We now sort the list such that we minimize the denominators when
        // dividing fOSC by the N2_HS candidate. This is in order to minimize
        // N31, which allows us to keep f3 as high as possible.
        std::stable_sort(
            N2_HS_candidates.begin(), N2_HS_candidates.end(),
            [&fOSC](uint32_t a, uint32_t b) {
              return (fOSC / a).denominator() < (fOSC / b).denominator();
            });

        for (auto N2_HS : N2_HS_candidates) {
          auto const f3_N2 = fOSC / (2 * N2_HS);
          auto const N31_cand = f3_N2.denominator();

          if (N31_cand > N3_MAX) {
            continue;
          }

          // Compute the upper limit for the GPS frequency.
          auto const gps_hi
              = std::min<int64_t>(limits.GPS_HI, N31_cand * limits.F3_HI);
          int64_t N2_LS_cand = 2;
          int64_t fGPS;

          if (f3_N2.numerator() <= gps_hi) {
            fGPS = f3_N2.numerator();
          } else {
            // Find the largest factor in f3_N2's numerator that is less than
            // gps_hi.
            fGPS = largest_factor(f3_N2.numerator(), gps_hi);
            N2_LS_cand *= f3_N2.numerator() / fGPS;
          }

          if (N2_LS_cand > N2_LS_MAX
              or static_cast<double>(fGPS) / N31_cand < limits.F3_LO) {
            continue;
          }

          // We have found a new possible solution.
          solution sol{
              .fGPS = boost::numeric_cast<uint32_t>(fGPS),
              .N31 = boost::numeric_cast<uint32_t>(N31_cand),
              .N1_HS = N1_HS,
              .NC1_LS = boost::numeric_cast<uint32_t>(NC1_LS),
              .NC2_LS = boost::numeric_cast<uint32_t>(NC2_LS),
              .N2_HS = N2_HS,
              .N2_LS = boost::numeric_cast<uint32_t>(N2_LS_cand),
          };

          if (found and algorithm != find::all) {
            if (sol < solutions[0]) {
              solutions[0] = sol;
            }
          } else {
            solutions.emplace_back(sol);
          }

          if (algorithm != find::all) {
            auto f3r = N31_cand * limits.F3_HI;
            auto fnd = f3r == fGPS       ? find::best
                       : f3r <= fGPS * 2 ? find::good
                                         : find::any;

            if (!found or fnd > *found) {
              found = fnd;
            }

            if (*found >= algorithm) {
              break;
            }
          }
        }
      }

      if (found and (*found >= algorithm)) {
        break;
      }
    }

    if (found and (*found >= algorithm)) {
      break;
    }
  }

  if (solutions.size() > 1) {
    std::stable_sort(solutions.begin(), solutions.end());
  }

  return solutions;
}

} // namespace gpsdo_config
