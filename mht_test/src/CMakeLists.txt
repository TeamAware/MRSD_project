#set( APP_NAME multi-track-example)

add_executable(mht_test 
  main_msg.cpp
  )

target_link_libraries(mht_test 
  ${Boost_LIBRARIES}
  openmht-common
  openmht-filter
  openmht-multi
  openmht-plot
  )

# add_executable(mht_msg 
#   main_msg.cpp
#   )

# target_link_libraries(mht_msg 
#   ${Boost_LIBRARIES}
#   openmht-common
#   openmht-filter
#   openmht-multi
#   openmht-plot
#   )
