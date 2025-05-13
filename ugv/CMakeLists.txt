PROJECT(uavcoop_ugv)
cmake_minimum_required(VERSION 2.8)

SET(FLAIR_USE_FILTER TRUE)
SET(FLAIR_USE_SENSOR_ACTUATOR TRUE)
SET(FLAIR_USE_META TRUE)
SET(FLAIR_USE_VRPN TRUE)

include($ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake)

SET(SRC_FILES
	src/main.cpp
	src/UgvNavigation.cpp
        ./src/tcp_optitrack/optitrack_server.cpp
)

install(DIRECTORY resources/core2-64/
        DESTINATION $ENV{FLAIR_ROOT}/flair-install/bin/demos/core2-64/uavcoop
        FILES_MATCHING
)


FLAIR_DEMO(${PROJECT_NAME} "${SRC_FILES}"
           DEST_DIR uavcoop
           COPY_RESOURCES
)
