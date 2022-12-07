# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "lidar_mapping: 0 messages, 3 services")

set(MSG_I_FLAGS "")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(lidar_mapping_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/generate_map_srv.srv" NAME_WE)
add_custom_target(_lidar_mapping_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lidar_mapping" "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/generate_map_srv.srv" ""
)

get_filename_component(_filename "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/start_builder_srv.srv" NAME_WE)
add_custom_target(_lidar_mapping_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lidar_mapping" "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/start_builder_srv.srv" ""
)

get_filename_component(_filename "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/save_map_srv.srv" NAME_WE)
add_custom_target(_lidar_mapping_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lidar_mapping" "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/save_map_srv.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(lidar_mapping
  "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/generate_map_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_mapping
)
_generate_srv_cpp(lidar_mapping
  "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/start_builder_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_mapping
)
_generate_srv_cpp(lidar_mapping
  "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/save_map_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_mapping
)

### Generating Module File
_generate_module_cpp(lidar_mapping
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_mapping
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(lidar_mapping_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(lidar_mapping_generate_messages lidar_mapping_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/generate_map_srv.srv" NAME_WE)
add_dependencies(lidar_mapping_generate_messages_cpp _lidar_mapping_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/start_builder_srv.srv" NAME_WE)
add_dependencies(lidar_mapping_generate_messages_cpp _lidar_mapping_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/save_map_srv.srv" NAME_WE)
add_dependencies(lidar_mapping_generate_messages_cpp _lidar_mapping_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lidar_mapping_gencpp)
add_dependencies(lidar_mapping_gencpp lidar_mapping_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lidar_mapping_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(lidar_mapping
  "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/generate_map_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_mapping
)
_generate_srv_eus(lidar_mapping
  "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/start_builder_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_mapping
)
_generate_srv_eus(lidar_mapping
  "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/save_map_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_mapping
)

### Generating Module File
_generate_module_eus(lidar_mapping
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_mapping
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(lidar_mapping_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(lidar_mapping_generate_messages lidar_mapping_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/generate_map_srv.srv" NAME_WE)
add_dependencies(lidar_mapping_generate_messages_eus _lidar_mapping_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/start_builder_srv.srv" NAME_WE)
add_dependencies(lidar_mapping_generate_messages_eus _lidar_mapping_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/save_map_srv.srv" NAME_WE)
add_dependencies(lidar_mapping_generate_messages_eus _lidar_mapping_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lidar_mapping_geneus)
add_dependencies(lidar_mapping_geneus lidar_mapping_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lidar_mapping_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(lidar_mapping
  "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/generate_map_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_mapping
)
_generate_srv_lisp(lidar_mapping
  "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/start_builder_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_mapping
)
_generate_srv_lisp(lidar_mapping
  "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/save_map_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_mapping
)

### Generating Module File
_generate_module_lisp(lidar_mapping
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_mapping
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(lidar_mapping_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(lidar_mapping_generate_messages lidar_mapping_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/generate_map_srv.srv" NAME_WE)
add_dependencies(lidar_mapping_generate_messages_lisp _lidar_mapping_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/start_builder_srv.srv" NAME_WE)
add_dependencies(lidar_mapping_generate_messages_lisp _lidar_mapping_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/save_map_srv.srv" NAME_WE)
add_dependencies(lidar_mapping_generate_messages_lisp _lidar_mapping_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lidar_mapping_genlisp)
add_dependencies(lidar_mapping_genlisp lidar_mapping_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lidar_mapping_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(lidar_mapping
  "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/generate_map_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_mapping
)
_generate_srv_nodejs(lidar_mapping
  "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/start_builder_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_mapping
)
_generate_srv_nodejs(lidar_mapping
  "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/save_map_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_mapping
)

### Generating Module File
_generate_module_nodejs(lidar_mapping
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_mapping
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(lidar_mapping_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(lidar_mapping_generate_messages lidar_mapping_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/generate_map_srv.srv" NAME_WE)
add_dependencies(lidar_mapping_generate_messages_nodejs _lidar_mapping_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/start_builder_srv.srv" NAME_WE)
add_dependencies(lidar_mapping_generate_messages_nodejs _lidar_mapping_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/save_map_srv.srv" NAME_WE)
add_dependencies(lidar_mapping_generate_messages_nodejs _lidar_mapping_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lidar_mapping_gennodejs)
add_dependencies(lidar_mapping_gennodejs lidar_mapping_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lidar_mapping_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(lidar_mapping
  "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/generate_map_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_mapping
)
_generate_srv_py(lidar_mapping
  "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/start_builder_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_mapping
)
_generate_srv_py(lidar_mapping
  "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/save_map_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_mapping
)

### Generating Module File
_generate_module_py(lidar_mapping
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_mapping
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(lidar_mapping_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(lidar_mapping_generate_messages lidar_mapping_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/generate_map_srv.srv" NAME_WE)
add_dependencies(lidar_mapping_generate_messages_py _lidar_mapping_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/start_builder_srv.srv" NAME_WE)
add_dependencies(lidar_mapping_generate_messages_py _lidar_mapping_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alorli/WORK_SPACE_LCJ/catkin_ws/src/lidar_mapping/srv/save_map_srv.srv" NAME_WE)
add_dependencies(lidar_mapping_generate_messages_py _lidar_mapping_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lidar_mapping_genpy)
add_dependencies(lidar_mapping_genpy lidar_mapping_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lidar_mapping_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_mapping)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_mapping
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_mapping)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_mapping
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_mapping)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_mapping
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_mapping)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_mapping
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_mapping)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_mapping\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_mapping
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
