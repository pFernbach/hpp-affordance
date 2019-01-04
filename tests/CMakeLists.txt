# Copyright 2016 CNRS-LAAS
#
# Author: Anna Seppala
#
# This file is part of hpp-affordance
# hpp-affordance is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# hpp-affordance is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Lesser Public License for more details.
# You should have received a copy of the GNU Lesser General Public License
# along with hpp-affordance  If not, see <http://www.gnu.org/licenses/>.

# Make Boost.Test generates the main function in test cases.
#ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

SET(${PROJECT_NAME}_TESTS
  operations
  one-triangle1
  one-triangle2
  oriented-triangles1
  oriented-triangles2
  )

FOREACH(test ${${PROJECT_NAME}_TESTS})
  ADD_UNIT_TEST(${test} "test-${test}")
  PKG_CONFIG_USE_DEPENDENCY(${test} hpp-fcl)
  PKG_CONFIG_USE_DEPENDENCY(${test} eigen3)
  target_link_libraries(${test} ${Boost_LIBRARIES} ${PROJECT_NAME})
ENDFOREACH(test)
