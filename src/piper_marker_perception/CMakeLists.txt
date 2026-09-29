cmake_minimum_required(VERSION 3.16)
project(piper_marker_perception)

find_package(ament_cmake REQUIRED)

# aruco_board.py and press_targets.py are libraries the other scripts import;
# the rest are entry points. All of it is Python because the work is I/O plus a
# few matrix operations, and because detector thresholds get tuned by editing,
# not by recompiling.
install(FILES scripts/aruco_board.py
        scripts/press_targets.py
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/make_board.py
        scripts/detect_board.py
        scripts/selftest_offline.py
        scripts/selftest_bridge.py
        scripts/press_target_bridge.py
        scripts/teach_press_offset.py
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
