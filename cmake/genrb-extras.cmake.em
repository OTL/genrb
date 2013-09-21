@[if DEVELSPACE]@
# location of scripts in develspace
set(GENRB_BIN_DIR "@(CMAKE_CURRENT_SOURCE_DIR)/scripts")
@[else]@
# location of scripts in installspace
set(GENRB_BIN_DIR "${genrb_DIR}/../../../@(CATKIN_PACKAGE_BIN_DESTINATION)")
@[end if]@

set(GENMSG_RB_BIN ${GENRB_BIN_DIR}/genmsg_rb.py)
set(GENSRV_RB_BIN ${GENRB_BIN_DIR}/gensrv_rb.py)

# Generate .msg->.h for rb
# The generated .h files should be added ALL_GEN_OUTPUT_FILES_rb
macro(_generate_msg_rb ARG_PKG ARG_MSG ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)

  #Append msg to output dir
  set(GEN_OUTPUT_DIR "${ARG_GEN_OUTPUT_DIR}")
  file(MAKE_DIRECTORY ${GEN_OUTPUT_DIR})
  #Create input and output filenames
  get_filename_component(MSG_SHORT_NAME ${ARG_MSG} NAME_WE)

  set(MSG_GENERATED_NAME ${MSG_SHORT_NAME}.rb)
  set(GEN_OUTPUT_FILE ${GEN_OUTPUT_DIR}/${MSG_GENERATED_NAME})

  add_custom_command(OUTPUT ${GEN_OUTPUT_FILE}
    DEPENDS ${GENMSG_RB_BIN} ${ARG_MSG} ${ARG_MSG_DEPS}
    COMMAND ${CATKIN_ENV} ${GENMSG_RB_BIN} ${ARG_MSG}
    ${ARG_IFLAGS}
    -p ${ARG_PKG}
    -o ${GEN_OUTPUT_DIR}
    COMMENT "Generating Ruby from MSG ${ARG_PKG}/${MSG_SHORT_NAME}"
    )

  list(APPEND ALL_GEN_OUTPUT_FILES_rb ${GEN_OUTPUT_FILE})

endmacro()

#todo, these macros are practically equal. Check for input file extension instead
macro(_generate_srv_rb ARG_PKG ARG_SRV ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)

  #Append msg to output dir
  set(GEN_OUTPUT_DIR "${ARG_GEN_OUTPUT_DIR}")
  file(MAKE_DIRECTORY ${GEN_OUTPUT_DIR})

  #Create input and output filenames
  get_filename_component(SRV_SHORT_NAME ${ARG_SRV} NAME_WE)

  set(SRV_GENERATED_NAME _${SRV_SHORT_NAME}.rb)
  set(GEN_OUTPUT_FILE ${GEN_OUTPUT_DIR}/${SRV_GENERATED_NAME})

  add_custom_command(OUTPUT ${GEN_OUTPUT_FILE}
    DEPENDS ${GENSRV_RB_BIN} ${ARG_SRV} ${ARG_MSG_DEPS}
    COMMAND ${CATKIN_ENV} ${RBTHON_EXECUTABLE} ${GENSRV_RB_BIN} ${ARG_SRV}
    ${ARG_IFLAGS}
    -p ${ARG_PKG}
    -o ${GEN_OUTPUT_DIR}
    COMMENT "Generating Ruby code from SRV ${ARG_PKG}/${SRV_SHORT_NAME}"
    )

  list(APPEND ALL_GEN_OUTPUT_FILES_rb ${GEN_OUTPUT_FILE})

endmacro()

macro(_generate_module_rb ARG_PKG ARG_GEN_OUTPUT_DIR ARG_GENERATED_FILES)
endmacro()

set(ruby_INSTALL_DIR lib/ruby/vendor_ruby)
set(genrb_INSTALL_DIR ${ruby_INSTALL_DIR})
