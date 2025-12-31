# UserCode/CMakeLists.txt
cmake_minimum_required(VERSION 3.21)

set(LIB_NAME OPS_driver)

project(${LIB_NAME})

# 导库方式
# add_subdirectory({project_path}/UserCode)
# target_link_libraries(${PROJECT_NAME}.elf PRIVATE OPS_driver)

# ---------------------------------------------------------------------------
# guard: UserCode should only be included by parent project
# ---------------------------------------------------------------------------
if (PROJECT_IS_TOP_LEVEL)
    message(FATAL_ERROR "UserCode must be included via add_subdirectory() from parent project")
endif ()

# ---------------------------------------------------------------------------
# layer options
# ---------------------------------------------------------------------------
#option(ENABLE_BSP "enable BSP layer" ON)

# ---------------------------------------------------------------------------
# collect layer sources
# ---------------------------------------------------------------------------
set(ALL_SOURCES drivers/ops.c)

set(ALL_HEADERS "drivers/ops.h")

# ---------------------------------------------------------------------------
# feature options: option + source + macro
# ---------------------------------------------------------------------------
set(DRIVER_MACROS "")

# ---------------------------------------------------------------------------
# create static library
# ---------------------------------------------------------------------------
add_library(OPS_driver STATIC ${ALL_SOURCES})
if (NOT TARGET ${LIB_NAME})
    message(FATAL_ERROR "target ${LIB_NAME} not created")
endif ()

# ---------------------------------------------------------------------------
# compile definitions
# ---------------------------------------------------------------------------
if (DRIVER_MACROS)
    target_compile_definitions(${LIB_NAME} PUBLIC ${DRIVER_MACROS})
endif ()

# ---------------------------------------------------------------------------
# build/include 目录
# ---------------------------------------------------------------------------
set(EXPORT_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include)
file(MAKE_DIRECTORY ${EXPORT_INCLUDE_DIR})
target_include_directories(${LIB_NAME} PUBLIC ${EXPORT_INCLUDE_DIR})

# ---------------------------------------------------------------------------
# copy selected headers
# ---------------------------------------------------------------------------
foreach (header IN LISTS ALL_HEADERS)
    configure_file(${CMAKE_CURRENT_LIST_DIR}/${header}
            ${EXPORT_INCLUDE_DIR}/${header} COPYONLY)
endforeach ()


# ---------------------------------------------------------------------------
# declare dependencies
# ---------------------------------------------------------------------------
add_dependencies(${PROJECT_NAME} stm32cubemx)
target_link_libraries(${PROJECT_NAME} PRIVATE stm32cubemx)

add_dependencies(${PROJECT_NAME} FreeRTOS)
target_link_libraries(${PROJECT_NAME} PRIVATE FreeRTOS)

target_link_libraries(${PROJECT_NAME} PRIVATE m)