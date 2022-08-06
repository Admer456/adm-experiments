
cmake_minimum_required( VERSION 3.16 )

project( AdmExperiments CXX C )

set_property( GLOBAL PROPERTY USE_FOLDERS ON )

## C++17 cuz' adm-utils
set( CMAKE_CXX_STANDARD 17 )

## I have a habit of setting a root variable cuz' I'm lazy to type CMAKE_CURRENT_SOURCE_DIR every time
## In projects like these, which aren't meant to be used as dependencies, I prefix stuff with THE_,
## cuz' it's THE stuff, there won't be any other
set( THE_ROOT ${CMAKE_CURRENT_SOURCE_DIR} )

## Set up adm-utils, debug-draw, SDL2, GLEW and GLM

## adm-utils
add_subdirectory( extern/adm-utils )
set_target_properties( AdmUtils PROPERTIES
    FOLDER "Libs" )

## SDL2
if( UNIX )
    find_package( SDL2 REQUIRED )
elseif( WIN32 )
    ## Note that I've only included 64-bit SDL2 binaries
    set( SDL2_INCLUDE_DIRS
        ${THE_ROOT}/extern/SDL2/include )

    set( SDL2_LIBRARIES
        ${THE_ROOT}/extern/SDL2/lib/SDL2.lib
        ${THE_ROOT}/extern/SDL2/lib/SDL2main.lib )
else()
    message( FATAL_ERROR "This platform is not supported" )
endif()

## OpenGL
find_package( OpenGL REQUIRED )

## GLM
set( GLM_INCLUDE_DIRS
    ${THE_ROOT}/extern/glm )

## GLEW
set( GLEW_INCLUDE_DIRS
	${THE_ROOT}/extern/glew/include )

## debug-draw
set( DEBUGDRAW_INCLUDE_DIRS
	${THE_ROOT}/extern/debug-draw )

set( COMMON_SOURCES
	${THE_ROOT}/experiments/common/DebugDrawBackend.cpp
	${THE_ROOT}/experiments/common/DebugDrawBackend.hpp
	${THE_ROOT}/experiments/common/IApplication.hpp
	${THE_ROOT}/experiments/common/Launcher.cpp )

function(set_up_example EXAMPLE_NAME EXAMPLE_SOURCES)

	message( "Generating project ${EXAMPLE_NAME} with: ${EXAMPLE_SOURCES}" )
	set( SOURCES
		${EXAMPLE_SOURCES}
		${COMMON_SOURCES}
		${THE_ROOT}/extern/glew/src/glew.c )

	# The .exe
	add_executable( ${EXAMPLE_NAME} ${SOURCES} )

	target_compile_definitions( ${EXAMPLE_NAME} PRIVATE -DGLEW_STATIC )

	# Include dirs
	target_include_directories( ${EXAMPLE_NAME} PRIVATE
		${THE_ROOT}
		AdmUtils
		OpenGL::GL
		${SDL2_INCLUDE_DIRS}
		${GLM_INCLUDE_DIRS}
		${DEBUGDRAW_INCLUDE_DIRS}
		${GLEW_INCLUDE_DIRS} )

	# Link against needed libraries
	target_link_libraries( ${EXAMPLE_NAME} PRIVATE ${SDL2_LIBRARIES} ${OPENGL_LIBRARIES} AdmUtils OpenGL::GL )

	# Output here
	install( TARGETS ${EXAMPLE_NAME}
		RUNTIME DESTINATION ${THE_ROOT}/bin
		LIBRARY DESTINATION ${THE_ROOT}/bin )

	## On Windows, copy SDL2.dll and the .pdb
	if( WIN32 )
		install( FILES ${THE_ROOT}/extern/SDL2/lib/SDL2.dll
			DESTINATION ${THE_ROOT}/bin/ )
		
		install( FILES $<TARGET_PDB_FILE:${EXAMPLE_NAME}> DESTINATION ${THE_ROOT}/bin/ OPTIONAL )
	endif()

endfunction(set_up_example)

add_subdirectory( experiments/octree )
