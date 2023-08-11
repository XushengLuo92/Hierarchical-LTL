file(GLOB src_files qplib/*.cc qplib/*.cpp)
file(GLOB header_files qplib/*.hh qplib/*.hpp)

INCLUDE_DIRECTORIES(qplib)

# set(library_src_files ${src_files})
# list(REMOVE_ITEM library_src_files ${CMAKE_CURRENT_SOURCE_DIR}/main.cc)
add_library(qplib SHARED ${src_files} ${header_files})
# set_property(TARGET qplib PROPERTY POSITION_INDEPENDENT_CODE 1)
# set_property(TARGET qplib PROPERTY PUBLIC_HEADER ${header_files})

install(TARGETS qplib EXPORT qplibConfig
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION include/qplib)

install(EXPORT qplibConfig DESTINATION cmake)
