cmake_minimum_required(VERSION 3.20)
project(aubo_telecommunication_cpp LANGUAGES CXX)

if(NOT MSVC OR NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR "Use Visual Studio with the x64 platform for the bundled Windows SDK.")
endif()
#设置sdk的目录以及把变量保存到Cmake缓存中，后续配置可以继续使用，PATH：说明这是一个目录路径。
set(AUBO_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/sdk" CACHE PATH "Official AUBO SDK directory")
#aubo_sdk是要查找的包名，CONFIG：使用 SDK 提供的 CMake 包配置文件
#REQUIRED：	找不到就报错并停止，NO_DEFAULT_PATH：禁用默认搜索路径，避免找到其他位置安装的 SDK
#加载成功后，就可以使用 SDK 定义的目标，例如后面的 aubo_sdk::aubo_sdk
find_package(aubo_sdk CONFIG REQUIRED
    PATHS "${AUBO_SDK_DIR}/lib/cmake/aubo_sdk" NO_DEFAULT_PATH)

add_executable(robot_communication main.cpp)#添加可执行文件
#编译这个程序时，编译器至少提供 C++17 支持，PRIVATE表示不作为接口要求向依赖它的其他目标传播
target_compile_features(robot_communication PRIVATE cxx_std_17)
target_compile_options(robot_communication PRIVATE /utf-8 /W4)
#让程序依赖并链接 SDK
target_link_libraries(robot_communication PRIVATE aubo_sdk::aubo_sdk)

#复制哪个文件：SDK 的 DLL 完整路径。
#复制到哪里：robot_communication.exe 所在目录。
add_custom_command(TARGET robot_communication POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:aubo_sdk::aubo_sdk>" "$<TARGET_FILE_DIR:robot_communication>"
    VERBATIM)
