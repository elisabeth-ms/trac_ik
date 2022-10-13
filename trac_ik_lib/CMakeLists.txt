cmake_minimum_required(VERSION 2.8.3)
project(trac_ik_lib)




#find_package(catkin REQUIRED
#  COMPONENTS
#    cmake_modules
#    kdl_parser
#    roscpp
#    urdf
#)
find_package(orocos_kdl 1.4)

find_package(Boost REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pkg_nlopt REQUIRED nlopt)
pkg_check_modules(Eigen REQUIRED eigen3)
# TODO: resolve libraries to absolute paths

      set(CMAKE_THREAD_LIBS_INIT "-lpthread")
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
      set(CMAKE_HAVE_THREADS_LIBRARY 1)
      set(CMAKE_USE_WIN32_THREADS_INIT 0)
      set(CMAKE_USE_PTHREADS_INIT 1)
      set(CMAKE_POSITION_INDEPENDENT_CODE ON)
      set(THREADS_PREFER_PTHREAD_FLAG ON)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${pkg_nlopt_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_library(trac_ik
  src/kdl_tl.cpp
  src/nlopt_ik.cpp
  src/trac_ik.cpp)

target_link_libraries(trac_ik
  nlopt
  ${pkg_nlopt_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES})


target_include_directories(trac_ik PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  ${orocos_kdl_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
  ${nlopt_INCLUDE_DIRS}
  ${track_ik_INCLUDE_DIRS})

install(TARGETS trac_ik
  ARCHIVE DESTINATION  ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION  ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION $ ${CMAKE_INSTALL_LIBDIR}
)


add_executable(example src/example.cpp)

target_link_libraries(example PRIVATE
                                 nlopt
                                 trac_ik
                                 ${orocos_kdl_LIBRARIES}
                                 )

target_include_directories(example PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                          $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
                                          ${orocos_kdl_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
                                          ${nlopt_INCLUDE_DIRS}
                                          ${track_ik_INCLUDE_DIRS})

target_include_directories(example PRIVATE ${Boost_INCLUDE_DIRS})


install(DIRECTORY include/   DESTINATION /usr/include)

