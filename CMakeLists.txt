cmake_minimum_required(VERSION 3.15)
project(LOGGER LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

include(FindThreads)
include(FetchContent)

FetchContent_Declare(chalk GIT_REPOSITORY https://github.com/ebenali/chalk)
FetchContent_MakeAvailable(chalk)

FetchContent_Declare(hhdate GIT_REPOSITORY https://github.com/HowardHinnant/date)
FetchContent_MakeAvailable(hhdate)	

FetchContent_Declare(fmt GIT_REPOSITORY https://github.com/fmtlib/fmt)
FetchContent_MakeAvailable(fmt)

include(FindThreads)
add_library(liblogger INTERFACE)
target_link_libraries(liblogger INTERFACE chalk::chalk date::date Threads::Threads fmt::fmt)
target_include_directories(liblogger INTERFACE "${CMAKE_CURRENT_LIST_DIR}")
target_compile_options(liblogger INTERFACE -Wno-deprecated-declarations)
target_sources(liblogger INTERFACE logutil.h)

set(LIBLOGGER_COMPILE_EXAMPLE CACHE BOOL ON)
if(LIBLOGGER_COMPILE_EXAMPLE)
	add_executable(example example.cc)
	target_link_libraries(example PRIVATE liblogger)
endif()
