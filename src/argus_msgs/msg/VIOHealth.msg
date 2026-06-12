# VIOHealth — health/quality report from the ARGUS VIO estimator (Pillar 2).
# Consumed by the perception-aware planner (Pillar 4) to gate behaviour when VIO
# degrades (e.g. featureless Zone B) and logged for the paper's failure analysis.
# Publish rate: matches the VIO output rate (nominally the camera rate, 30 Hz).

std_msgs/Header header

# --- Estimator status -------------------------------------------------------
uint8 STATUS_INITIALIZING=0   # filter not yet converged
uint8 STATUS_NOMINAL=1        # healthy tracking
uint8 STATUS_DEGRADED=2       # tracking weak; pose usable but uncertain
uint8 STATUS_LOST=3           # tracking failed; pose not trustworthy
uint8 status

# --- Confidence -------------------------------------------------------------
float32 confidence            # [0..1] normalised health score (1.0 = best)

# --- Feature tracking -------------------------------------------------------
uint16 num_tracked_features   # features tracked in the current frame
uint16 num_inlier_features    # inliers retained after RANSAC / optimisation
float32 avg_parallax          # px, mean feature parallax (triangulation strength)

# --- Uncertainty ------------------------------------------------------------
float32 position_covariance_trace   # m^2, trace of the 3x3 position covariance
float32 estimated_drift_rate        # m/s, running drift estimate vs. expectation

# --- Observability / timing -------------------------------------------------
bool imu_excitation_ok        # true if recent motion excites the IMU (scale observable)
float32 processing_latency_ms # ms, estimator compute time for the current frame
