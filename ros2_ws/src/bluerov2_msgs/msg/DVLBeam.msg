# One acoustic beam (transducer) of a Water Linked DVL.
# Mirrors an entry of the "transducers" array in the DVL-A50 JSON velocity report.

uint8 id                # Transducer index as reported by the DVL (0..3 on an A50)
float64 velocity        # Along-beam velocity [m/s], positive away from the transducer
float64 distance        # Along-beam slant range to the reflector [m]
float64 rssi            # Received signal strength indicator [dBm]
float64 nsd             # Noise spectral density [dBm]
bool beam_valid         # True when the DVL considers this beam's lock trustworthy
