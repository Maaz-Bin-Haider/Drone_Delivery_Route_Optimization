"""Physical and operational constants for the simulation.

Units are fixed throughout the system and never mixed:
distance in kilometres, speed in metres per second, time in minutes,
energy as a percentage of a full battery charge.
"""

EARTH_RADIUS_KM = 6371.0088

# --- Flight model (TDD section 6.1) ---------------------------------------
V_AIR_MS = 15.0     # nominal cruise airspeed in still air (54 km/h)
V_MIN_MS = 3.0      # ground-speed floor; keeps time finite in extreme headwind
W_MAX_MS = 20.0     # maximum wind speed the model accepts

# Bounds on the wind energy multiplier. Outside this band the constant-power
# cruise approximation stops being defensible, so the multiplier is clamped.
MU_MIN = 0.70
MU_MAX = 1.60

# --- Energy model ---------------------------------------------------------
ENERGY_RATE_PCT_PER_KM = 3.5    # still-air consumption; ~28 km nominal range
CHARGE_RATE_PCT_PER_MIN = 4.0   # recharge rate at a charging station
RESERVE_PCT = 15.0              # safety reserve, never spent by a planned route

# --- Operations -----------------------------------------------------------
SERVICE_TIME_MIN = 2.0          # fixed handover time at a destination

EPS = 1e-9
