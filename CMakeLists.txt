add_eff_app(Avionics-Argus3-SDR
    TYPE exe
    ARCHS e1x
    TARGETS scalar
    SOURCE main.c gnss_types.c
)

target_link_libraries(Avionics-Argus3-SDR PRIVATE m)
