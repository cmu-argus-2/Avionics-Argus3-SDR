add_eff_app(Avionics-Argus3-SDR
    TYPE exe
    ARCHS e1x
    TARGETS scalar
    SOURCE efficient_spi_rx_stub.c gnss_types.c
)
