Test SDR GPS to locate satellite. The efficient chip would be collecting digitized GNSS radio samples and outputting per-satellite timing/frequency measurements needed to determine the CubeSat’s own position, velocity, and time. 

raw samples (iq16_t) -> mix_down (gnss_sat16) -> correlate -> metric (gnss_mag2_iq32) -> tracking (gnss_iabs32) -> track_state_t -> gnss_measurement_t -> UART output
