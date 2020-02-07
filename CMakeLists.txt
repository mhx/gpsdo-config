#
# Copyright (c) 2020 Marcus Holland-Moritz (github@mhxnet.de)
#
# This file is part of gpsdo-config.
#
# gpsdo-config is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# gpsdo-config is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with gpsdo-config.  If not, see <https://www.gnu.org/licenses/>.
#

PROJECT(gpsdo-config)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(CMAKE_BUILD_TYPE release)

ENABLE_TESTING()
INCLUDE(GoogleTest)

FIND_PACKAGE(Boost 1.62 REQUIRED COMPONENTS
             program_options)

INCLUDE_DIRECTORIES(${BOOST_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall")

ADD_LIBRARY(gpsdo_solver
            solver
           )

ADD_EXECUTABLE(gpsdo-config
               main
              )

ADD_EXECUTABLE(solver_test
               test/solver_test
              )

TARGET_LINK_LIBRARIES(gpsdo-config
                      gpsdo_solver
                      Boost::program_options
                     )

TARGET_LINK_LIBRARIES(solver_test
                      gpsdo_solver
                      gtest
                      gtest_main
                     )

gtest_discover_tests(solver_test)

INSTALL(TARGETS gpsdo-config
        RUNTIME DESTINATION bin)
