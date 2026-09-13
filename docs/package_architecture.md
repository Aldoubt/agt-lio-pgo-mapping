# Package Architecture

Phase 1 / Commit 1 establishes buildable package boundaries only. No package contains a LiDAR adapter, LIO, PGO, exporter implementation, launch executable, or copied source from `agt_navigation_v3`.

```text
agt_mapping_bringup
  ├── agt_mapping_core
  ├── agt_mid360_adapter
  ├── agt_fastlio_backend ──> agt_mapping_frontend_api
  ├── agt_mapping_exporter
  └── agt_mapping_artifacts
            │
            └── agt_mapping_interfaces
```

| Package | Build type | Responsibility in later commits |
| --- | --- | --- |
| `agt_mapping_interfaces` | `ament_cmake` | Stable ROS messages/services/actions for mapping status and artifact export. Empty in Commit 1. |
| `agt_mapping_core` | `ament_python` | Session configuration, lifecycle coordination and backend-neutral pipeline state. |
| `agt_mapping_bringup` | `ament_python` | Composition launch package; will eventually expose `mapping_v0.launch.py`. |
| `agt_mapping_artifacts` | `ament_python` | Artifact schema validation, provenance and integrity helpers. |
| `agt_mid360_adapter` | `ament_cmake` | MID360 `CustomMsg` to PointCloud2 adapter; it preserves header time/frame and maps reflectivity to intensity. |
| `agt_mapping_frontend_api` | `ament_cmake` | Backend-neutral frontend output topic contract for odometry, body cloud and path. |
| `agt_fastlio_backend` | `ament_cmake` | Relays external FAST-LIO2 odometry, body cloud and path into the frontend contract without changing messages. |
| `agt_mapping_exporter` | `ament_python` | Map/trajectory/patch export orchestration and package-level output validation. |

The `.repos` file pins external FAST-LIO2, PGO and HBA together in `fast_lio2_mapping`, plus Batch-LIO separately. These are future external dependencies, not packages included in the Commit 1 build graph.
