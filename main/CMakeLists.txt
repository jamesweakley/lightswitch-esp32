idf_component_register(SRC_DIRS "." "./lights"
                       PRIV_INCLUDE_DIRS
                         "." "./lights" "${ESP_MATTER_PATH}/examples/common/utils")

set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD 17)
target_compile_options(${COMPONENT_LIB} PRIVATE "-DCHIP_HAVE_CONFIG_H")

# ---- Internal CHIP (Matter SDK) headers for direct CommandSender usage ----
# NOTE: These are considered internal APIs and may change between releases.
# They are added to enable explicit unicast command sending while esp-matter
# lacks a public helper for this use-case.
set(MATTER_SDK_PATH "$ENV{ESP_MATTER_PATH}/connectedhomeip/connectedhomeip")
target_include_directories(${COMPONENT_LIB} PRIVATE
  "${MATTER_SDK_PATH}/src/app"
  "${MATTER_SDK_PATH}/src/lib/core"
  "${MATTER_SDK_PATH}/src/lib/support"
  "${MATTER_SDK_PATH}/src/transport"
  "${MATTER_SDK_PATH}/src/messaging"
)