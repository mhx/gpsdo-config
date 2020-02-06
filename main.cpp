/*
 * GPSDO Configuration Tool
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

#include <cctype>
#include <iostream>
#include <stdexcept>
#include <string>

#include <boost/program_options.hpp>

#include "solver.h"

namespace {

namespace po = boost::program_options;

gpsdo_config::rat64 parse_fraction(std::string const& str) {
  int64_t num = 0, den = 1, integral = 0, unit = 1;
  bool decimal = false, blank = false, frac = false;

  for (auto s : str) {
    if (s == '.') {
      if (decimal or blank or frac) {
        throw std::invalid_argument("invalid input");
      }
      decimal = true;
    } else if (s == ' ' || s == '_') {
      if (decimal or blank or frac) {
        throw std::invalid_argument("invalid input");
      }
      blank = true;
      integral = num;
      num = 0;
    } else if (s == '/') {
      if (decimal or frac) {
        throw std::invalid_argument("invalid input");
      }
      frac = true;
      den = 0;
    } else if (s == 'k') {
      if (unit != 1) {
        throw std::invalid_argument("invalid input");
      }
      unit = 1'000;
    } else if (s == 'M') {
      if (unit != 1) {
        throw std::invalid_argument("invalid input");
      }
      unit = 1'000'000;
    } else if (std::isdigit(s)) {
      int dig = (s - '0');

      if (frac) {
        den = den * 10 + dig;
      } else {
        num = num * 10 + dig;

        if (decimal) {
          den *= 10;
        }
      }
    } else {
      throw std::invalid_argument("invalid input");
    }
  }

  if (den == 0) {
    throw std::invalid_argument("invalid input");
  }

  return (gpsdo_config::rat64(num, den) + integral) * unit;
}

void gpsdo_usage(
    std::ostream& os, char const* prog, po::options_description const& desc) {
  os << "Usage: " << prog << " f1 [f2] [options...]"
     << "\n\n"
     << desc << "\n"
     << "If only one frequency is specified, both outputs will be set to the\n"
     << "same frequency. Frequencies will be processed accurately as rational\n"
     << "numbers internally, and can also be specified as such. An integral\n"
     << "part can be separated from a fraction by either a single space or an\n"
     << "underscore. Suffixes `M` and `k` are supported for MHz and kHz.\n\n"
     << "`--all` and `--best` can be really slow as there may be millions\n"
     << "of possible solutions. By default, the code will look for a \"good\"\n"
     << "solution, which shouldn't be significantly slower than `--any`.\n"
     << "The \"quality\" of a solution is measured purely by means of the\n"
     << "phase detector comparison frequency (f3), which directly impacts\n"
     << "jitter/phase noise. `--best` will always search for the solution\n"
     << "with the highest possible f3. The default behaviour will accept\n"
     << "any f3 that is higher than 50\% of the maximum value.\n\n"
     << "Output for `--json` and `--cmdline` will always be exclusively\n"
     << "written to stdout, suitable for processing by other commands.\n"
     << "All other output will be written to stderr.\n\n"
     << "Examples:\n"
     << "  " << prog << " 1000\n"
     << "  " << prog << " 10M 96k\n"
     << "  " << prog << " 1000.31 2345.61 --best\n"
     << "  " << prog << " 10_1/7k 500/9k --all --verbose\n"
     << "  lb-gps-linux /dev/hidraw3 $(" << prog << " 10M 120M --cmdline)\n\n"
     << "Exit status:\n"
     << "  0: successful completion\n"
     << "  1: could not find any solution for the specified frequencies\n"
     << "  2: input processing error\n\n";
}

int gpsdo_main(int argc, char** argv) {
  using namespace gpsdo_config;

  bool find_all = false, find_any = false, find_best = false, verbose = false,
       cmdline = false, json = false;
  std::string f1_str, f2_str;

  po::options_description desc("Options");
  // clang-format off
  desc.add_options()
      ("f1", po::value<std::string>(&f1_str), "frequency 1")
      ("f2", po::value<std::string>(&f2_str), "frequency 2")
      ("all", po::bool_switch(&find_all), "find all possible solutions")
      ("any", po::bool_switch(&find_any), "find any possible solution")
      ("best", po::bool_switch(&find_best), "find best possible solution")
      ("verbose,v", po::bool_switch(&verbose), "print more information")
      ("cmdline", po::bool_switch(&cmdline), "print command line config")
      ("json", po::bool_switch(&json), "print solutions as json objects")
      ("help,h", "produce help message");
  // clang-format on

  auto error = [&](std::string const& err) {
    std::cerr << "ERROR: " << err << "\n\n";
    gpsdo_usage(std::cerr, argv[0], desc);
  };

  po::positional_options_description pos;
  pos.add("f1", 1);
  pos.add("f2", 1);

  po::variables_map vm;

  try {
    po::store(
        po::command_line_parser(argc, argv).options(desc).positional(pos).run(),
        vm);
  } catch (std::exception const& e) {
    error(e.what());
    return 2;
  }

  po::notify(vm);

  if (vm.count("help")) {
    gpsdo_usage(std::cout, argv[0], desc);
    return 0;
  }

  if (f1_str.empty()) {
    error("at least one frequency must be specified");
    return 2;
  }

  if ((find_all + find_any + find_best) > 1) {
    error("only one of --any, --best, --all can be specified");
    return 2;
  }

  if ((cmdline + json) > 1) {
    error("only one of --cmdline, --json can be specified");
    return 2;
  }

  rat64 f1, f2;

  f1 = parse_fraction(f1_str);

  if (f2_str.empty()) {
    f2 = f1;
  } else {
    f2 = parse_fraction(f2_str);
  }

  hardware_limits limits{
      // Source: Silicon Labs Si53xx-RM Rev. 1.3, Table 26
      .VCO_LO = 4'850'000'000,
      .VCO_HI = 5'670'000'000,
      .F3_LO = 2'000,
      .F3_HI = 2'000'000,

      // Source: ublox MAX-M8 series data sheet
      .GPS_HI = 10'000'000,
  };

  auto solutions = find_solutions(
      f1, f2, limits,
      find_all ? find::all
               : find_any ? find::any : find_best ? find::best : find::good);

  if (solutions.empty()) {
    std::cerr << "no solutions found" << std::endl;
    return 1;
  }

  if (verbose or find_all) {
    std::cerr << "found " << solutions.size() << " solution(s)" << std::endl;
  }

  for (auto const& s : solutions) {
    if (verbose or (!cmdline and !json)) {
      s.write(std::cerr, verbose);
      std::cerr << std::endl;
    }
    if (cmdline) {
      std::cout << "--gps " << s.fGPS << " --n31 " << s.N31 << " --n2_ls "
                << s.N2_LS << " --n2_hs " << s.N2_HS << " --n1_hs " << s.N1_HS
                << " --nc1_ls " << s.NC1_LS << " --nc2_ls " << s.NC2_LS
                << std::endl;
    }
    if (json) {
      std::cout << "{\"fGPS\": " << s.fGPS << ", \"N31\": " << s.N31
                << ", \"N2_LS\": " << s.N2_LS << ", \"N2_HS\": " << s.N2_HS
                << ", \"N1_HS\": " << s.N1_HS << ", \"NC1_LS\": " << s.NC1_LS
                << ", \"NC2_LS\": " << s.NC2_LS << "}" << std::endl;
    }
  }

  return 0;
}

} // namespace

int main(int argc, char** argv) {
  try {
    return gpsdo_main(argc, argv);
  } catch (std::exception const& e) {
    std::cerr << "ERROR: " << e.what() << std::endl;
    return 2;
  }
}
