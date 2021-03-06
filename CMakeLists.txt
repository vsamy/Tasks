# Copyright 2012-2016 CNRS-UM LIRMM, CNRS-AIST JRL
#
# This file is part of Tasks.
#
# Tasks is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Tasks is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with Tasks.  If not, see <http://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 2.8)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)

SET(PROJECT_NAME Tasks)
SET(PROJECT_DESCRIPTION "...")
SET(PROJECT_URL "https://github.com/jorisv//Tasks")
SET(PROJECT_DEBUG_POSTFIX "_d")

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)
set(DOXYGEN_USE_MATHJAX "YES")

SETUP_PROJECT(FALSE)
add_definitions(-mfpmath=sse -msse2)

option(PYTHON_BINDING "Generate python binding." OFF)
option(UNIT_TESTS "Generate unit tests." ON)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++0x -pedantic")
endif()

#########################
# External dependencies
SEARCH_FOR_EIGEN()

ADD_REQUIRED_DEPENDENCY("sch-core")
ADD_REQUIRED_DEPENDENCY("SpaceVecAlg")
ADD_REQUIRED_DEPENDENCY("RBDyn" "RBDyn_d")
ADD_REQUIRED_DEPENDENCY("eigen-qld")
ADD_OPTIONAL_DEPENDENCY("eigen-lssol")

#########################
#

add_subdirectory(src)
add_subdirectory(tests)

if(${PYTHON_BINDING})
 add_subdirectory(binding/python)
endif()

# Add dependency towards the library in the pkg-config file.
PKG_CONFIG_APPEND_LIBS(Tasks)
SETUP_PROJECT_FINALIZE()
