cmake_minimum_required(VERSION 3.20)
project(aubo_telecommunication_cpp LANGUAGES CXX)

if(NOT MSVC OR NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR "Use Visual Studio with the x64 platform for the Windows SDK.")
endif()

# V4.5 controllers use the legacy service on TCP 8899, not the ARCS RPC SDK.
set(AUBO_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/sdk" CACHE PATH "Legacy AUBO Windows x64 SDK directory")
foreach(required include/rsdef.h lib/libserviceinterface.lib bin/libserviceinterface.dll)
    if(NOT EXISTS "${AUBO_SDK_DIR}/${required}")
        message(FATAL_ERROR "Missing legacy SDK file: ${AUBO_SDK_DIR}/${required}. See README.md.")
    endif()
endforeach()

add_library(aubo_legacy SHARED IMPORTED)
set_target_properties(aubo_legacy PROPERTIES
    IMPORTED_IMPLIB "${AUBO_SDK_DIR}/lib/libserviceinterface.lib"
    IMPORTED_LOCATION "${AUBO_SDK_DIR}/bin/libserviceinterface.dll"
    INTERFACE_INCLUDE_DIRECTORIES "${AUBO_SDK_DIR}/include")

add_executable(robot_communication main.cpp)
target_compile_features(robot_communication PRIVATE cxx_std_17)
target_compile_options(robot_communication PRIVATE /utf-8 /W4)
target_link_libraries(robot_communication PRIVATE aubo_legacy)

# The vendor DLL uses MinGW internally. Only its C API with POD types crosses
# the ABI boundary; do not pass std::vector/string or use its C++ classes here.
file(GLOB sdk_runtime_dlls CONFIGURE_DEPENDS "${AUBO_SDK_DIR}/bin/*.dll")
add_custom_command(TARGET robot_communication POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${sdk_runtime_dlls} "$<TARGET_FILE_DIR:robot_communication>"
    COMMAND_EXPAND_LISTS
    VERBATIM)
