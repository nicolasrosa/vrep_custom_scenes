cmake_minimum_required(VERSION 2.6)
project(cppremoteapi)

INCLUDE(FindPkgConfig)

IF(CMAKE_COMPILER_IS_GNUCC)
SET(CMAKE_CXX_FLAGS
"${CMAKE_CXX_FLAGS} -Wall -std=c++0x -DBUILD_SHARED_LIBS=OFF -lrt")
ENDIF(CMAKE_COMPILER_IS_GNUCC)

# definicoes da remoteAPI para CPP
add_definitions(-DNON_MATLAB_PARSING)
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)
add_definitions(-DDO_NOT_USE_SHARED_MEMORY)

set(SOURCES
    remoteApi/extApi.c
    #remoteApi/extApiCustom.c
    remoteApi/extApiPlatform.c
    # common/shared_memory.c
)
 
set(HEADERS
    remoteApi/extApi.h
    #remoteApi/extApiCustom.h
    remoteApi/extApiInternal.h
    remoteApi/extApiPlatform.h
    include/simConst.h
    include/simLib.h
    # include/shared_memory.h
)
# include_directories("common/")
include_directories("include/")
include_directories("remoteApi/")

find_package (Threads)

add_executable(cppremoteapi 
               src/main.cpp 
               ${SOURCES}
	           ${HEADERS})
	       
target_link_libraries (cppremoteapi ${CMAKE_THREAD_LIBS_INIT})

install(TARGETS cppremoteapi RUNTIME DESTINATION bin)
