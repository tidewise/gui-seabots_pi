##---------------------------------------------------------------------------
## Author:      Sean D'Epagnier
## Copyright:
## License:     GPLv3+
##---------------------------------------------------------------------------

# define minimum cmake version
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.2)

PROJECT(seabots_pi)
SET(PACKAGE_NAME seabots_pi)
SET(VERBOSE_NAME SeaBots)
SET(TITLE_NAME SeaBots)
SET(CPACK_PACKAGE_CONTACT "Sylvain Joyeux")

INCLUDE("cmake/PluginConfigure.cmake")

SET(SRC_SeaBots src/Plugin.cpp)

list(APPEND IMAGES images/seabots.png)
add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/Images.cpp ${CMAKE_CURRENT_BINARY_DIR}/Images.hpp
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/tools/png2wx.pl
        -C ${CMAKE_CURRENT_BINARY_DIR}/Images.cpp
        -H ${CMAKE_CURRENT_BINARY_DIR}/Images.hpp
        -M SEABOTS_PI_IMAGES_HPP ${IMAGES}
    DEPENDS ${IMAGES} ${CMAKE_CURRENT_SOURCE_DIR}/tools/png2wx.pl
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

#INCLUDE("cmake/PluginNMEA.cmake")
#INCLUDE("cmake/PluginJSON.cmake")
#INCLUDE("cmake/PluginGL.cmake")

add_library(seabots_pi MODULE ${SRC_SeaBots} ${CMAKE_CURRENT_BINARY_DIR}/Images.cpp)
target_include_directories(seabots_pi PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
add_definitions(-DTIXML_USE_STL)

INCLUDE("cmake/PluginInstall.cmake")
INCLUDE("cmake/PluginLocalization.cmake")
INCLUDE("cmake/PluginPackage.cmake")
