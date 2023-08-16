#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "qplib" for configuration ""
set_property(TARGET qplib APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(qplib PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libqplib.so"
  IMPORTED_SONAME_NOCONFIG "libqplib.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS qplib )
list(APPEND _IMPORT_CHECK_FILES_FOR_qplib "${_IMPORT_PREFIX}/lib/libqplib.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
