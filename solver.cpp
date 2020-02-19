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

rat64 rat_lcm(rat64 r1, rat64 r2) {
  int64_t den = boost::integer::lcm(r1.denominator(), r2.denominator());
  int64_t num = boost::integer::lcm(
      r1.numerator() * (den / r1.denominator()),
      r2.numerator() * (den / r2.denominator()));
  return rat64(num, den);
}

bool is_div_one_or_even(rat64 a, rat64 b) {
  return a == b or boost::rational_cast<int64_t>(a / b) % 2 == 0;
}

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

bool solution::operator<(solution const& rhs) const {
  return static_cast<double>(fGPS) / N31
         > static_cast<double>(rhs.fGPS) / rhs.N31;
}

std::vector<solution> find_solutions(
    rat64 f1, rat64 f2, hardware_limits const& limits, find algorithm) {
  int64_t const N12_LS_MAX = 1 << 20;
  int64_t const N3_MAX = 1 << 19;
  auto lcm_freq = rat_lcm(f1, f2);
  auto q_max
      = boost::rational_cast<int64_t>(N12_LS_MAX * std::max(f1, f2) / lcm_freq);

  if (!is_div_one_or_even(f1, lcm_freq) or !is_div_one_or_even(f2, lcm_freq)) {
    // NCn_LS must be 1 or even
    lcm_freq *= 2;
  }

  auto base_div1 = boost::rational_cast<int64_t>(lcm_freq / f1);
  auto base_div2 = boost::rational_cast<int64_t>(lcm_freq / f2);

  std::unordered_set<rat64> fOSC_seen;
  std::vector<solution> solutions;

  std::optional<find> found;

  for (uint32_t N1_HS = 11; N1_HS >= 4; --N1_HS) {
    auto f_N1 = N1_HS * lcm_freq;
    auto q_lo = std::min(
        q_max, static_cast<int64_t>(std::ceil(
                   boost::rational_cast<double>(limits.VCO_LO / f_N1))));
    auto q_hi = std::min(
        q_max, static_cast<int64_t>(std::floor(
                   boost::rational_cast<double>(limits.VCO_HI / f_N1))));

    for (uint32_t q = q_lo; q <= q_hi; ++q) {
      int64_t const NC1_LS = q * base_div1;
      int64_t const NC2_LS = q * base_div2;

      if (is_in_ncx_ls_range(NC1_LS) and is_in_ncx_ls_range(NC2_LS)) {
        auto fOSC = lcm_freq * q * N1_HS;

        if (fOSC_seen.insert(fOSC).second) {
          std::array<uint32_t, 8> n2_hs_val;
          std::iota(n2_hs_val.rbegin(), n2_hs_val.rend(), 4);
          std::stable_sort(
              n2_hs_val.begin(), n2_hs_val.end(),
              [&fOSC](uint32_t a, uint32_t b) {
                return (fOSC / a).denominator() < (fOSC / b).denominator();
              });

          for (auto N2_HS : n2_hs_val) {
            auto f3_n2 = fOSC / (2 * N2_HS);
            auto N31_cand = f3_n2.denominator();

            if (N31_cand <= N3_MAX) {
              auto gps_hi
                  = std::min<int64_t>(limits.GPS_HI, N31_cand * limits.F3_HI);
              int64_t N2_LS_cand = 2;
              int64_t fGPS;

              if (f3_n2.numerator() <= gps_hi) {
                fGPS = f3_n2.numerator();
              } else {
                fGPS = largest_factor(f3_n2.numerator(), gps_hi);
                N2_LS_cand *= f3_n2.numerator() / fGPS;
              }

              if (N2_LS_cand <= N12_LS_MAX
                  and static_cast<double>(fGPS) / N31_cand >= limits.F3_LO) {
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
                  auto fnd = f3r == fGPS
                                 ? find::best
                                 : f3r <= fGPS * 2 ? find::good : find::any;
                  if (!found or fnd > *found) {
                    found = fnd;
                  }
                  if (*found >= algorithm) {
                    break;
                  }
                }
              }
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
