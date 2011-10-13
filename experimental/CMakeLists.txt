CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(CoreSLAM)

IF(MSVC)
    ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS)
ENDIF(MSVC)

IF(MSVC)
  SET(libs winmm)
ENDIF(MSVC)

IF(CMAKE_HOST_UNIX)
  SET(libs m)
  OPTION(MULTICORE_SUPPORT "Multicore experimental support" OFF)
  IF(MULTICORE_SUPPORT)
      ADD_DEFINITIONS(-DMULTICORE_SUPPORT)
      SET(libs ${libs} pthread)
  ENDIF(MULTICORE_SUPPORT)
ENDIF(CMAKE_HOST_UNIX)

OPTION(COMP_32BITS "32 bits target" OFF)
IF(COMP_32BITS)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m32")
ENDIF(COMP_32BITS)

OPTION(USE_SSE3 "Use SSE3 intructions" ON)
OPTION(CORE2 "Compile for Core2 target" OFF)
OPTION(ATOM "Compile for Atom target" OFF)

IF(USE_SSE3)
    ADD_DEFINITIONS(-DUSE_SSE3)
    IF (CORE2)
        SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=core2")
    ELSE(CORE2)
        IF (ATOM)
            SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=atom")
        ELSE(ATOM)
            IF(APPLE)
                SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=core2")
            ELSE(APPLE)
                IF(CMAKE_HOST_UNIX)
                    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native")
                ENDIF(CMAKE_HOST_UNIX)
            ENDIF(APPLE)
        ENDIF(ATOM)
    ENDIF(CORE2)
ENDIF(USE_SSE3)

OPTION(USE_SDL "Use SDL for visualization" ON)
IF(USE_SDL)
    ADD_DEFINITIONS(-DUSE_SDL)
    IF(MSVC)
        FIND_LIBRARY(SDLLIB NAMES SDL PATHS "../corebots/third_parties/SDL/win32/lib")
        FIND_LIBRARY(SDLmainLIB NAMES SDLmain PATHS "../corebots/third_parties/SDL/win32/lib")
        FIND_LIBRARY(SDLGFXLIB NAMES SDL_gfx PATHS "../corebots/third_parties/SDL_gfx/win32/lib")
        FIND_PATH(SDL_INCLUDE NAMES SDL.h PATHS "../corebots/third_parties/SDL/win32/include")
        FIND_PATH(GFX_INCLUDE NAMES SDL_rotozoom.h PATHS "../corebots/third_parties/SDL_gfx/win32/include")
        INCLUDE_DIRECTORIES(${SDL_INCLUDE} ${GFX_INCLUDE})
    ENDIF(MSVC)
    IF(CMAKE_HOST_UNIX)
        FIND_PACKAGE(SDL REQUIRED)
        FIND_LIBRARY(SDLGFXLIB NAMES SDL_gfx PATHS "/usr/local/lib")
    ENDIF(CMAKE_HOST_UNIX)
ELSE(USE_SDL)
    SET(SDL_LIBRARY "")
    SET(SDLGFXLIB "")
    SET(SDLLIB "")
    SET(SDLmainLIB "")
ENDIF(USE_SDL)

SET(sources
    core_slam.h
    core_slam_internals.h
    core_slam.c
    core_slam_dmap.c
    core_slam_ext.c
    core_slam_map_update.c
    core_slam_pqueue.c
    core_slam_matrix.c
)

ADD_LIBRARY(CoreSLAM SHARED ${sources})
INSTALL(TARGETS CoreSLAM ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin LIBRARY DESTINATION lib)
INSTALL(FILES core_slam.h DESTINATION include)

ADD_LIBRARY(CoreSLAMClient SHARED core_slam_client.c core_slam_client_dmap.c)
INSTALL(TARGETS CoreSLAMClient ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin LIBRARY DESTINATION lib)
INSTALL(FILES core_slam_client.h DESTINATION include)

# Packaging info
SET(CPACK_PACKAGE_NAME "CoreSLAM")
SET(CPACK_PACKAGE_VERSION_MAJOR 1)
SET(CPACK_PACKAGE_VERSION_MINOR 2)
SET(CPACK_PACKAGE_VERSION_PATCH 0)
SET(CPACK_PACKAGE_FILE_NAME "CoreSLAM-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")

SET(CPACK_PACKAGE_VENDOR "ARMINES / Mines ParisTech")
#SET(CPACK_RESOURCE_FILE_README ${CMAKE_CURRENT_SOURCE_DIR}/README.txt)

# Packaging info
SET(CPACK_GENERATOR ZIP)
IF(UNIX)
  SET(CPACK_GENERATOR ${CPACK_GENERATOR};STGZ;TGZ)
ENDIF(UNIX)
IF(LINUX)
SET(CPACK_GENERATOR ${CPACK_GENERATOR};DEB;RPM)
ENDIF(LINUX)
IF(MSVC)
  SET(CPACK_GENERATOR ${CPACK_GENERATOR};NSIS)
ENDIF(MSVC)
IF(APPLE)
  SET(CPACK_GENERATOR ${CPACK_GENERATOR};PackageMaker)
ENDIF(APPLE)

INCLUDE(CPack)
