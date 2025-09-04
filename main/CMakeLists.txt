idf_component_register(SRC_DIRS "." "./lights" "./temp"
                       PRIV_INCLUDE_DIRS
                         "." "./lights" "${ESP_MATTER_PATH}/examples/common/utils")

set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD 17)
target_compile_options(${COMPONENT_LIB} PRIVATE "-DCHIP_HAVE_CONFIG_H")

# Internal CHIP (Matter SDK) headers needed for CommandSender (unicast send callback)
set(MATTER_SDK_PATH "$ENV{ESP_MATTER_PATH}/connectedhomeip/connectedhomeip")
target_include_directories(${COMPONENT_LIB} PRIVATE
  "${MATTER_SDK_PATH}/src/app"
  "${MATTER_SDK_PATH}/src/lib/core"
  "${MATTER_SDK_PATH}/src/lib/support"
  "${MATTER_SDK_PATH}/src/transport"
  "${MATTER_SDK_PATH}/src/messaging"
)