# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "fcu_common: 7 messages, 0 services")

set(MSG_I_FLAGS "-Ifcu_common:/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(fcu_common_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_State.msg" NAME_WE)
add_custom_target(_fcu_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fcu_common" "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_State.msg" ""
)

get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_Waypoint.msg" NAME_WE)
add_custom_target(_fcu_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fcu_common" "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_Waypoint.msg" ""
)

get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_Controller_Commands.msg" NAME_WE)
add_custom_target(_fcu_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fcu_common" "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_Controller_Commands.msg" ""
)

get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/GPS.msg" NAME_WE)
add_custom_target(_fcu_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fcu_common" "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/GPS.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/MR_Controller_Commands.msg" NAME_WE)
add_custom_target(_fcu_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fcu_common" "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/MR_Controller_Commands.msg" "geometry_msgs/Vector3:std_msgs/Header"
)

get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/Command.msg" NAME_WE)
add_custom_target(_fcu_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fcu_common" "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/Command.msg" ""
)

get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_Current_Path.msg" NAME_WE)
add_custom_target(_fcu_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fcu_common" "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_Current_Path.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(fcu_common
  "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fcu_common
)
_generate_msg_cpp(fcu_common
  "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fcu_common
)
_generate_msg_cpp(fcu_common
  "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_Controller_Commands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fcu_common
)
_generate_msg_cpp(fcu_common
  "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/GPS.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fcu_common
)
_generate_msg_cpp(fcu_common
  "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/MR_Controller_Commands.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fcu_common
)
_generate_msg_cpp(fcu_common
  "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/Command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fcu_common
)
_generate_msg_cpp(fcu_common
  "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_Current_Path.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fcu_common
)

### Generating Services

### Generating Module File
_generate_module_cpp(fcu_common
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fcu_common
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(fcu_common_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(fcu_common_generate_messages fcu_common_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_State.msg" NAME_WE)
add_dependencies(fcu_common_generate_messages_cpp _fcu_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_Waypoint.msg" NAME_WE)
add_dependencies(fcu_common_generate_messages_cpp _fcu_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_Controller_Commands.msg" NAME_WE)
add_dependencies(fcu_common_generate_messages_cpp _fcu_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/GPS.msg" NAME_WE)
add_dependencies(fcu_common_generate_messages_cpp _fcu_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/MR_Controller_Commands.msg" NAME_WE)
add_dependencies(fcu_common_generate_messages_cpp _fcu_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/Command.msg" NAME_WE)
add_dependencies(fcu_common_generate_messages_cpp _fcu_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_Current_Path.msg" NAME_WE)
add_dependencies(fcu_common_generate_messages_cpp _fcu_common_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fcu_common_gencpp)
add_dependencies(fcu_common_gencpp fcu_common_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fcu_common_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(fcu_common
  "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fcu_common
)
_generate_msg_lisp(fcu_common
  "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fcu_common
)
_generate_msg_lisp(fcu_common
  "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_Controller_Commands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fcu_common
)
_generate_msg_lisp(fcu_common
  "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/GPS.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fcu_common
)
_generate_msg_lisp(fcu_common
  "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/MR_Controller_Commands.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fcu_common
)
_generate_msg_lisp(fcu_common
  "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/Command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fcu_common
)
_generate_msg_lisp(fcu_common
  "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_Current_Path.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fcu_common
)

### Generating Services

### Generating Module File
_generate_module_lisp(fcu_common
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fcu_common
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(fcu_common_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(fcu_common_generate_messages fcu_common_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_State.msg" NAME_WE)
add_dependencies(fcu_common_generate_messages_lisp _fcu_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_Waypoint.msg" NAME_WE)
add_dependencies(fcu_common_generate_messages_lisp _fcu_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_Controller_Commands.msg" NAME_WE)
add_dependencies(fcu_common_generate_messages_lisp _fcu_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/GPS.msg" NAME_WE)
add_dependencies(fcu_common_generate_messages_lisp _fcu_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/MR_Controller_Commands.msg" NAME_WE)
add_dependencies(fcu_common_generate_messages_lisp _fcu_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/Command.msg" NAME_WE)
add_dependencies(fcu_common_generate_messages_lisp _fcu_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_Current_Path.msg" NAME_WE)
add_dependencies(fcu_common_generate_messages_lisp _fcu_common_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fcu_common_genlisp)
add_dependencies(fcu_common_genlisp fcu_common_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fcu_common_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(fcu_common
  "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fcu_common
)
_generate_msg_py(fcu_common
  "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fcu_common
)
_generate_msg_py(fcu_common
  "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_Controller_Commands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fcu_common
)
_generate_msg_py(fcu_common
  "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/GPS.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fcu_common
)
_generate_msg_py(fcu_common
  "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/MR_Controller_Commands.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fcu_common
)
_generate_msg_py(fcu_common
  "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/Command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fcu_common
)
_generate_msg_py(fcu_common
  "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_Current_Path.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fcu_common
)

### Generating Services

### Generating Module File
_generate_module_py(fcu_common
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fcu_common
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(fcu_common_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(fcu_common_generate_messages fcu_common_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_State.msg" NAME_WE)
add_dependencies(fcu_common_generate_messages_py _fcu_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_Waypoint.msg" NAME_WE)
add_dependencies(fcu_common_generate_messages_py _fcu_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_Controller_Commands.msg" NAME_WE)
add_dependencies(fcu_common_generate_messages_py _fcu_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/GPS.msg" NAME_WE)
add_dependencies(fcu_common_generate_messages_py _fcu_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/MR_Controller_Commands.msg" NAME_WE)
add_dependencies(fcu_common_generate_messages_py _fcu_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/Command.msg" NAME_WE)
add_dependencies(fcu_common_generate_messages_py _fcu_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jeff/Desktop/auvsi-client/catkin_ws/src/fcu_common/msg/FW_Current_Path.msg" NAME_WE)
add_dependencies(fcu_common_generate_messages_py _fcu_common_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fcu_common_genpy)
add_dependencies(fcu_common_genpy fcu_common_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fcu_common_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fcu_common)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fcu_common
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(fcu_common_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(fcu_common_generate_messages_cpp geometry_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fcu_common)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fcu_common
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(fcu_common_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(fcu_common_generate_messages_lisp geometry_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fcu_common)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fcu_common\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fcu_common
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(fcu_common_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(fcu_common_generate_messages_py geometry_msgs_generate_messages_py)
