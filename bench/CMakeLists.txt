# 示教、回放与仿真共用的离线工具，只由台架或测试构建选入。
if(NOT DEFINED CMAKE_RUNTIME_OUTPUT_DIRECTORY)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
endif()

add_library(scara_legacy_site legacy_site.cpp)
target_link_libraries(scara_legacy_site PUBLIC scara_motion)

add_executable(scara_geometry_tool ../tools/geometry_tool.cpp)
target_link_libraries(scara_geometry_tool PRIVATE scara_legacy_site)
add_executable(scara_game_sim ../tests/game_sim_io.cpp)
target_link_libraries(scara_game_sim PRIVATE scara_site scara_legacy_site)

if(WIN32 AND TARGET opencv_world)
  foreach(tool scara_geometry_tool scara_game_sim)
    add_custom_command(TARGET ${tool} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different
      $<TARGET_FILE:opencv_world> $<TARGET_FILE_DIR:${tool}>)
  endforeach()
endif()
