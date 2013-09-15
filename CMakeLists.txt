cmake_minimum_required(VERSION 2.8.3)
project(genrb)

find_package(catkin REQUIRED COMPONENTS genmsg)

catkin_python_setup()

catkin_package(
CATKIN_DEPENDS genmsg
CFG_EXTRAS genrb-extras.cmake
)

add_subdirectory(scripts)

file(WRITE ${CATKIN_DEVEL_PREFIX}/${GENMSG_LANGS_DESTINATION}/genrb "Ruby")
install(FILES ${CATKIN_DEVEL_PREFIX}/${GENMSG_LANGS_DESTINATION}/genrb
  DESTINATION ${GENMSG_LANGS_DESTINATION})
