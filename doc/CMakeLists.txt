cmake_minimum_required(VERSION 3.12)
project("esp-idf-tlv493d")

find_package(Python REQUIRED)

message(${Python_EXECUTABLE})

#file(READ "../include/version.h" ver)

#string(REGEX MATCH "#define VERSION_MAJOR ([0-9]*)" _ ${ver})
#set(ver_major ${CMAKE_MATCH_1})

#string(REGEX MATCH "#define VERSION_MINOR ([0-9]*)" _ ${ver})
#set(ver_minor ${CMAKE_MATCH_1})

#string(REGEX MATCH "#define VERSION_PATCH ([0-9]*)" _ ${ver})
#set(ver_patch ${CMAKE_MATCH_1})

#set(VERSION ${ver_major}.${ver_minor}.${ver_patch})

include (CheckFunctionExists)
check_function_exists (idf_component_register BUILD-IDF)

# look for Doxygen package
    find_package(Doxygen)

    # Source and header dir
    set(PROJECT_SOURCE_DIR "..")
    set(PROJECT_INCLUDE_DIR "../include")

    set(SRC_FILES tlv493d)

    foreach(file ${SRC_FILES})
	list(APPEND SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/${file}.cpp
        ${PROJECT_INCLUDE_DIR}/${file}.h
	)
    endforeach()
    message("Find files: ${SOURCE_FILES}")

    # Set up such that your IDE organizes the files correctly
    #source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCE_FILES})

    add_library(tlv493d ${SOURCE_FILES})

    set(CPACK_PROJECT_NAME ${PROJECT_NAME})
    include(CPack)
    
    if (DOXYGEN_FOUND)
        # set input and output files
        set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
        #set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile.out)
        set(DOXYGEN_OUT ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.out)


        set(SPHINX_SOURCE ${CMAKE_CURRENT_BINARY_DIR})
        set(SPHINX_BUILD ${CMAKE_BINARY_DIR}/sphinx/html)
        set(SPHINX_INDEX_FILE ${SPHINX_BUILD}/index.html)
        set(SPHINX_CONF_FILE ${CMAKE_BINARY_DIR})

        # request to configure the file
        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
        message("Doxygen build started")

        # Note: do not put "ALL" - this builds docs together with application EVERY TIME!
        add_custom_target( doc
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            COMMAND  ${Python_EXECUTABLE} -m sphinx -b html -c ${SPHINX_CONF_FILE}
            ${SPHINX_SOURCE} ${SPHINX_BUILD} -D version=0.2.0
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM )
    else (DOXYGEN_FOUND)
        message("Doxygen need to be installed to generate the doxygen documentation")
    endif (DOXYGEN_FOUND)
