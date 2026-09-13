# agt_pgo_backend

`agt_pgo_backend` is the bridge between the frontend and optimization backend contract. It consumes `/mapping/frontend/cloud` and `/mapping/frontend/odometry`, publishes:

- `/mapping/backend/map_pose` (`geometry_msgs/msg/PoseStamped`)
- `/mapping/backend/keyframes` (`nav_msgs/msg/Path`)
- `/mapping/backend/status` (`std_msgs/msg/String`)

It exposes `/mapping/backend/export_artifact` (`std_srvs/srv/Trigger`) as the artifact-export trigger consumed by a future exporter package.

The `pgo_backend.launch.py` launch can start the external `pgo` package only when `start_external_pgo:=true` and a compatible `pgo_config` is supplied. PGO stays a `.repos`-pinned external dependency; no PGO source is copied here.

The bridge's v0.1 keyframe stream is a deterministic distance-gated frontend observation stream. `map_pose` initially mirrors the frontend pose until external PGO correction output is wired into the contract; the status topic makes that integration state explicit (`ready_external_pgo`).
