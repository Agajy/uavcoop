PROJECT(uavcoop_simulator)
cmake_minimum_required(VERSION 2.8)

SET(FLAIR_USE_SIMULATOR TRUE)
SET(FLAIR_USE_SIMULATOR_GL TRUE)

include($ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake)

SET(SRC_FILES
	./src/main.cpp
	./src/ArenaGround.cpp
	./src/FourWheelRobot.cpp


)

install(DIRECTORY resources/core2-64/
        DESTINATION $ENV{FLAIR_ROOT}/flair-install/bin/demos/core2-64/uavcoop
        FILES_MATCHING
)

FLAIR_DEMO_HOST_CXX(${PROJECT_NAME} "${SRC_FILES}"
           DEST_DIR uavcoop
           COPY_RESOURCES
)
