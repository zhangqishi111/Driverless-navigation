cmake_minimum_required(VERSION 3.8)
project(landerpi_test)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  docs
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/launch_sim_all.sh
  scripts/record_core_bag.sh
  scripts/replay_core_bag.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(
    local_plan_frame_contract
    test/test_local_plan_frame_contract.py
  )

  add_test(
    NAME record_core_bag
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/test/test_record_core_bag.sh
  )

  add_test(
    NAME replay_core_bag
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/test/test_replay_core_bag.sh
  )
endif()

ament_package()
