# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "epos2_control: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iepos2_control:/home/pg/catkin_ws/src/epos2_control/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(epos2_control_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(epos2_control
  "/home/pg/catkin_ws/src/epos2_control/msg/HubFlow.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/epos2_control
)
_generate_msg_cpp(epos2_control
  "/home/pg/catkin_ws/src/epos2_control/msg/velocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/epos2_control
)

### Generating Services

### Generating Module File
_generate_module_cpp(epos2_control
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/epos2_control
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(epos2_control_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(epos2_control_generate_messages epos2_control_generate_messages_cpp)

# target for backward compatibility
add_custom_target(epos2_control_gencpp)
add_dependencies(epos2_control_gencpp epos2_control_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS epos2_control_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(epos2_control
  "/home/pg/catkin_ws/src/epos2_control/msg/HubFlow.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/epos2_control
)
_generate_msg_lisp(epos2_control
  "/home/pg/catkin_ws/src/epos2_control/msg/velocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/epos2_control
)

### Generating Services

### Generating Module File
_generate_module_lisp(epos2_control
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/epos2_control
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(epos2_control_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(epos2_control_generate_messages epos2_control_generate_messages_lisp)

# target for backward compatibility
add_custom_target(epos2_control_genlisp)
add_dependencies(epos2_control_genlisp epos2_control_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS epos2_control_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(epos2_control
  "/home/pg/catkin_ws/src/epos2_control/msg/HubFlow.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/epos2_control
)
_generate_msg_py(epos2_control
  "/home/pg/catkin_ws/src/epos2_control/msg/velocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/epos2_control
)

### Generating Services

### Generating Module File
_generate_module_py(epos2_control
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/epos2_control
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(epos2_control_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(epos2_control_generate_messages epos2_control_generate_messages_py)

# target for backward compatibility
add_custom_target(epos2_control_genpy)
add_dependencies(epos2_control_genpy epos2_control_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS epos2_control_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/epos2_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/epos2_control
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/epos2_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/epos2_control
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/epos2_control)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/epos2_control\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/epos2_control
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
