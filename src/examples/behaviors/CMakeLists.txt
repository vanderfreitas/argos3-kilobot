if(ARGOS_BUILD_FOR_SIMULATOR)
  include_directories(${CMAKE_SOURCE_DIR}/plugins/robots/kilobot/control_interface)
 
  #
  # ARK loop function: pcod_network
  #
  add_executable(pcod_network pcod_network.c)  
  target_link_libraries(pcod_network argos3plugin_simulator_kilolib)

  endif(ARGOS_BUILD_FOR_SIMULATOR)
