##---------------------------------------------------------------------------
## Author:      Sylvain Joyeux <sylvain.joyeux@tidewise.io>
## Copyright:
## License:     GPLv3+
##---------------------------------------------------------------------------

# define minimum cmake version
CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

find_package(Rock)
rock_init(seabots_pi 0.3)
SET(PACKAGE_NAME seabots_pi)
SET(VERBOSE_NAME SeaBots)
SET(TITLE_NAME SeaBots)
SET(CPACK_PACKAGE_CONTACT "Sylvain Joyeux")

INCLUDE("cmake/PluginConfigure.cmake")

rock_find_pkgconfig(BASE_TYPES base-types REQUIRED)

#INCLUDE("cmake/PluginNMEA.cmake")
#INCLUDE("cmake/PluginJSON.cmake")
#INCLUDE("cmake/PluginGL.cmake")

set(PLUGIN_DATA_PATH share/opencpn/plugins/seabots_pi)
configure_file(src/Paths.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/src/Paths.hpp @ONLY)

add_definitions(-DRTT_COMPONENT)
add_definitions(-DGL_GLEXT_PROTOTYPES)
find_package(marnav)
rock_library(seabots_pi
    MODULE src/Plugin.cpp src/NMEA.cpp src/OCPNInterfaceImpl.cpp
    DEPS_PLAIN OPENGL # OpenGL found by OCPN's PluginConfigure.cmake
    DEPS_PKGCONFIG base-types gps_base usv_control
        orocos-rtt-gnulinux
        orocos-rtt-corba-gnulinux
        std-typekit-gnulinux
        std-transport-corba-gnulinux
        std-transport-mqueue-gnulinux
        std-transport-typelib-gnulinux
        base-typekit-gnulinux
        base-transport-corba-gnulinux
        base-transport-mqueue-gnulinux
        base-transport-typelib-gnulinux
        gps_base-typekit-gnulinux
        gps_base-transport-corba-gnulinux
        gps_base-transport-mqueue-gnulinux
        gps_base-transport-typelib-gnulinux
        seabots_pi-tasks-gnulinux
        seabots_pi-typekit-gnulinux
        seabots_pi-transport-corba-gnulinux
        seabots_pi-transport-mqueue-gnulinux
        seabots_pi-transport-typelib-gnulinux
        logger-tasks-gnulinux
        logger-typekit-gnulinux
        logger-transport-corba-gnulinux
        logger-transport-mqueue-gnulinux
        logger-transport-typelib-gnulinux)
target_include_directories(seabots_pi PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/src)
target_link_libraries(seabots_pi marnav::marnav)

install(FILES images/seabots.svg
              images/plan_route.svg
              images/plan_route_toggled.svg
              images/execute_route.svg
              images/execute_route_toggled.svg
              src/trajectory.frag
              src/trajectory.vert
        DESTINATION "${PLUGIN_DATA_PATH}")

target_include_directories(seabots_pi PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
add_definitions(-DTIXML_USE_STL)

INCLUDE("cmake/PluginInstall.cmake")
INCLUDE("cmake/PluginLocalization.cmake")
INCLUDE("cmake/PluginPackage.cmake")

if (ROCK_TEST_ENABLED)
    add_subdirectory(test)
endif()