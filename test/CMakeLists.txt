INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/..)
LINK_DIRECTORIES(..)

if(CMAKE_HOST_UNIX)
set(libs m)
endif(CMAKE_HOST_UNIX)

OPTION(USE_SDL "Use SDL for visualization" ON)
IF(USE_SDL)
    ADD_DEFINITIONS(-DUSE_SDL)
    IF(CMAKE_HOST_UNIX)
        FIND_PACKAGE(SDL REQUIRED)
        FIND_LIBRARY(SDLGFXLIB NAMES SDL_gfx PATHS "/usr/local/lib")
        INCLUDE_DIRECTORIES("/usr/include/SDL")
    ENDIF(CMAKE_HOST_UNIX)
ELSE(USE_SDL)
    SET(SDLGFXLIB "")
    SET(SDLLIB "")
    SET(SDLmainLIB "")
ENDIF(USE_SDL)

add_executable(test_SDL test_loop_closing.c)
target_link_libraries(test_SDL CoreSLAM ${SDL_LIBRARY} ${SDLLIB} ${SDLmainLIB}
    ${SDLGFXLIB} ${libs})

add_executable(test_loop_closing test_loop_closing.c)
target_link_libraries(test_loop_closing CoreSLAM ${SDL_LIBRARY} ${SDLLIB} ${SDLmainLIB}
    ${SDLGFXLIB} ${libs})
file(COPY "./data" DESTINATION "." FILES_MATCHING PATTERN "*.dat")
file(COPY "." DESTINATION "." FILES_MATCHING PATTERN "*.bmp")

