#pragma once

/*
 * GPSDO Configuration Library
 *
 * Copyright (c) 2020 Marcus Holland-Moritz (github@mhxnet.de)
 *
 * This file is part of gpsdo-config.
 *
 * gpsdo_config is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * gpsdo_config is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with gpsdo_config.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <cstdint>
#include <ostream>
#include <vector>

#include <boost/rational.hpp>

namespace gpsdo_config {

using rat64 = boost::rational<int64_t>;

struct hardware_limits {
  int64_t VCO_LO;
  int64_t VCO_HI;
  int64_t F3_LO;
  int64_t F3_HI;
  int64_t GPS_HI;
};

struct solution {
  uint32_t fGPS;
  uint32_t N31;
  uint32_t N1_HS;
  uint32_t NC1_LS;
  uint32_t NC2_LS;
  uint32_t N2_HS;
  uint32_t N2_LS;

  void write(std::ostream& os, bool verbose = false) const;
  bool operator<(solution const&) const;
};

enum class find { any, good, best, all };

std::vector<solution> find_solutions(
    rat64 f1,
    rat64 f2,
    hardware_limits const& limits,
    find algorithm = find::any);

} // namespace gpsdo_config
