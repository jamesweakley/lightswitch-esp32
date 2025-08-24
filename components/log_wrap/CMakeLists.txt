idf_component_register(SRCS "log_wrap.cpp"
                       REQUIRES chip
                       PRIV_REQUIRES log)
# Ensure this component is linked after CHIP/openthread so wrappers resolve their refs
set_property(TARGET ${COMPONENT_LIB} PROPERTY LINK_INTERFACE_MULTIPLICITY 1)
