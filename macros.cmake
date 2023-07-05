# Adds a script (in a source file under the `scripts` directory) to the list of
# executables to compile. Adds the name to the list `SCRIPTS`.
macro(add_script script_name folder)
  list(APPEND SCRIPTS ${script_name})
  add_executable(${PROJECT_NAME}_${script_name} ${folder}/${script_name}.cpp)
  set_target_properties(${PROJECT_NAME}_${script_name} PROPERTIES OUTPUT_NAME ${script_name} PREFIX "")
  target_link_libraries(${PROJECT_NAME}_${script_name} ${LIBRARIES})
endmacro(add_script)

# Install scripts added via `add_script` to the install directory.
macro(install_scripts)
  install(TARGETS
    ${SCRIPTS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endmacro(install_scripts)

# Installs library to install directory. Assumes LIBRARY_NAME is set.
macro(install_library)
  install(TARGETS ${LIBRARY_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
endmacro(install_library)


