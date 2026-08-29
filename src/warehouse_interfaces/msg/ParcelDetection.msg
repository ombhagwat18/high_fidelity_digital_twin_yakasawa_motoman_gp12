# A single localized parcel on the conveyor, produced by an ArUco marker
# detection fused with the parcel catalog (marker ID -> shape/size/bin).

uint32 marker_id
string shape          # "box" | "cylinder" | "plate"
string bin            # target bin/pallet zone name, from the parcel catalog
float32[3] dims_m     # [length, width, height] in metres
geometry_msgs/Pose pose   # world-frame pick pose (position + gripper-down orientation)
float32 yaw               # marker's world-frame yaw (rad) — direct wrist alignment target
float32 confidence
