cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(gz-cmake3 REQUIRED)

project(Entities_control)

gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

gz_find_package(gz-common5 REQUIRED)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

add_library(EntitiesControl SHARED EntitiesControl)
set_property(TARGET EntitiesControl PROPERTY CXX_STANDARD 17)
target_link_libraries(EntitiesControl
        PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
        PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
        PRIVATE gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
        )