export HUSKY_DESCRIPTION=$(catkin_find --first-only --without-underlays husky_description urdf/husky.urdf.xacro 2>/dev/null)

export HUSKY_OUSTER_ENABLED=1
export HUSKY_LASER_3D_TOPIC=/ouster/points
export HUSKY_LASER_SCAN_TOPIC=/scan
