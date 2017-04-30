#set( APP_NAME multi-track-example)

add_executable(mht_test 
  main.cpp
  )

target_link_libraries(mht_test 
  ${Boost_LIBRARIES}
  openmht-common
  openmht-filter
  openmht-multi
  openmht-plot
  )
