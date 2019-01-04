SET(${PROJECT_NAME}_HEADERS
  fwd.hh
  affordance-extraction.hh
  operations.hh
 )

INSTALL(FILES
  ${${PROJECT_NAME}_HEADERS}
  DESTINATION include/hpp/affordance
  )
