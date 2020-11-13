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

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.11)

OPTION(WITH_TESTS "build with tests" OFF)

SET(CMAKE_BUILD_TYPE release)

if(WITH_TESTS)
  # Download and unpack googletest at configure time
  CONFIGURE_FILE(CMakeLists.txt.gtest googletest-download/CMakeLists.txt)
  EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
  if(result)
    MESSAGE(FATAL_ERROR "CMake step for googletest failed: ${result}")
  endif()
  EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} --build .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
  if(result)
    MESSAGE(FATAL_ERROR "Build step for googletest failed: ${result}")
  endif()

  # Prevent overriding the parent project's compiler/linker
  # settings on Windows
  SET(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

  # Add googletest directly to our build. This defines
  # the gtest and gtest_main targets.
  ADD_SUBDIRECTORY(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
                   ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
                   EXCLUDE_FROM_ALL)

  ENABLE_TESTING()
  INCLUDE(GoogleTest)
endif()

FIND_PACKAGE(Boost 1.58 REQUIRED COMPONENTS
             program_options)

INCLUDE_DIRECTORIES(${BOOST_INCLUDE_DIRS})

SET(CMAKE_CXX_STANDARD 20)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_EXTENSIONS OFF)

ADD_COMPILE_OPTIONS(-Wall -Wextra -pedantic)

ADD_LIBRARY(gpsdo_solver
            solver
           )

ADD_EXECUTABLE(gpsdo-config
               main
              )

TARGET_LINK_LIBRARIES(gpsdo-config
                      gpsdo_solver
                      Boost::program_options
                     )

if(WITH_TESTS)
  ADD_EXECUTABLE(solver_test
                 test/solver_test
                )

  TARGET_LINK_LIBRARIES(solver_test
                        gpsdo_solver
                        gtest_main
                       )

  gtest_discover_tests(solver_test)
endif()

INSTALL(TARGETS gpsdo-config
        RUNTIME DESTINATION bin)
