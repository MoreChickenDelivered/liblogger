cmake_minimum_required(VERSION 3.15)
project(liblogger LANGUAGES CXX VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_INCLUDE_CURRENT_DIR_IN_INTERFACE ON)
include(FetchContent)

add_library(liblogger INTERFACE)
add_library(liblogger::liblogger ALIAS liblogger)

find_package(chalk QUIET)
if (NOT TARGET chalk::chalk)
	FetchContent_Declare(chalk GIT_REPOSITORY https://github.com/ebenali/chalk)
	FetchContent_GetProperties(chalk)
	if (NOT chalk_POPULATED)
		FetchContent_Populate(chalk)
		add_subdirectory(${chalk_SOURCE_DIR} ${chalk_BINARY_DIR} EXCLUDE_FROM_ALL)
	endif()
endif()

find_package(date QUIET)
if (NOT TARGET date::date)
	FetchContent_Declare(hhdate GIT_REPOSITORY https://github.com/HowardHinnant/date)
	FetchContent_GetProperties(hhdate)
	if (NOT hhdate_POPULATED)
		FetchContent_Populate(hhdate)
		add_subdirectory(${hhdate_SOURCE_DIR} ${hhdate_BINARY_DIR} EXCLUDE_FROM_ALL)
	endif()
endif()

if(DEFINED ENV{VCPKG_ROOT})
	execute_process(COMMAND "$ENV{VCPKG_ROOT}/vcpkg" install --recurse
		boost-iostreams boost-iostreams[lzma]
		boost-lockfree
		liblzma bzip2 zstd cpptrace)
endif()

message(STATUS "[liblogger] Using system C++20 std::format")

if (NOT TARGET Threads::Threads)
	include(FindThreads)
endif()

set(CURL_LIBRARY "-lcurl")
find_package(CURL REQUIRED)

find_package(BZip2 REQUIRED)

find_package(LibLZMA REQUIRED)

find_package(zstd CONFIG REQUIRED)

find_package(ZLIB REQUIRED)

find_package(Boost REQUIRED COMPONENTS iostreams)

find_package(cpptrace CONFIG REQUIRED)

target_link_libraries(liblogger INTERFACE
	chalk::chalk date::date
	Threads::Threads ${CURL_LIBRARIES}
	Boost::boost Boost::iostreams
	LibLZMA::LibLZMA
	BZip2::BZip2
	$<IF:$<TARGET_EXISTS:zstd::libzstd_shared>,zstd::libzstd_shared,zstd::libzstd_static>
	ZLIB::ZLIB
	cpptrace::cpptrace
	)
target_include_directories(liblogger INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}> $<INSTALL_INTERFACE:include/liblogger>)
#target_compile_options(liblogger INTERFACE ${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations)
target_sources(liblogger INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/logutil/logutil.h>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/inspector/inspector.hpp>
	$<INSTALL_INTERFACE:>)
set_target_properties(liblogger PROPERTIES PUBLIC_HEADER "logutil.h;slack.h;inspector.hpp")

option(LIBLOGGER_COMPILE_EXAMPLE "Compile sample script" ON)

if(LIBLOGGER_COMPILE_EXAMPLE)
	add_executable(liblogger-example logutil/logutil_example.cc)
	target_link_libraries(liblogger-example PRIVATE liblogger -lrt)
	set_target_properties(liblogger-example PROPERTIES OUTPUT_NAME logutil_example)
	add_library(example_dylib SHARED logutil/logutil_example_dylib.cc)
	target_link_libraries(example_dylib PRIVATE liblogger  -lrt)
	target_compile_definitions(liblogger-example PRIVATE OVERRIDE_NDEBUG)
	target_compile_definitions(example_dylib PRIVATE OVERRIDE_NDEBUG)
endif()

#option(LIBLOGGER_INSTALL "Install liblogger" ON)
#if(LIBLOGGER_INSTALL)
	include(GNUInstallDirs)
	install(TARGETS liblogger EXPORT liblogger PUBLIC_HEADER DESTINATION include/liblogger)
	install(EXPORT liblogger NAMESPACE liblogger:: DESTINATION lib/cmake/liblogger)
#	export(EXPORT liblogger NAMESPACE liblogger:: FILE libloggerConfig.cmake)
#endif()
#
#
