#
# Copyright (c) 2016 CNRS
# Author: Anna Seppala
#
#
# This file is part of hpp-model
# hpp-model is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-model is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-model  If not, see
# <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(PROJECT_NAME hpp-affordance)
SET(PROJECT_DESCRIPTION "Implementation to Extract Whole-Body Affordances for Mutli-Contact Planning.")

SET(CXX_DISABLE_WERROR TRUE)

INCLUDE(cmake/hpp.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/test.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

SEARCH_FOR_BOOST()

ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.2")
ADD_REQUIRED_DEPENDENCY("hpp-fcl >= 0.2.9")

ADD_SUBDIRECTORY(include/hpp/affordance)
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

CONFIG_FILES (include/hpp/affordance/doc.hh)
PKG_CONFIG_APPEND_LIBS("hpp-affordance")
