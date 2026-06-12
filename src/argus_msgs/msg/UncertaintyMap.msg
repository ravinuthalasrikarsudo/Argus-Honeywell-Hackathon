# UncertaintyMap — 6-DoF pose uncertainty for the current VIO estimate (Pillar 2).
# The perception-aware planner (Pillar 4) propagates this covariance to weight
# candidate paths by expected localisation risk; the paper renders it as
# uncertainty ellipsoids along the flown trajectory.

std_msgs/Header header

# Pose the uncertainty is expressed about (same stamp/frame as header).
geometry_msgs/Pose pose

# 6x6 pose covariance, ROW-MAJOR, variable order [x, y, z, roll, pitch, yaw].
# Units: position m^2, orientation rad^2. Matches the geometry_msgs covariance
# convention (but float32 per the ARGUS contract, not float64).
float32[36] covariance

# Convenience scalars derived from `covariance` (planner fast-path / logging).
float32 position_uncertainty     # m,   1-sigma, sqrt of largest position eigenvalue
float32 orientation_uncertainty  # rad, 1-sigma, sqrt of largest orientation eigenvalue
