# AGT Mapping Product V1 — interfaces and extension guide

Status: code staged for review; ROS / GUI / real-bag acceptance NOT_RUN. This is
a composition layer over existing modules, NOT another map/schema validator.

## Ownership, in the order data flows

1. **Sensor input adapter**: MID360 CustomMsg + IMU only today. Sensor drivers
   and clock/calibration authority remain in the platform/sensor repositories.
   Bag/live input is validated before starting the session. Do not start another
   MID360 driver when the sensor owner already publishes these topics.
2. **Replaceable mapping frontend**: select a registered backend profile from
   agt_mapping_bringup.backend_registry. Each backend owns its native algorithm,
   config and tested export adapter. FAST-LIVO2 LIO-only is the sole currently
   verified session runner. Other registry profiles are references or research
   candidates, not accepted substitutions. No automatic FAST-LIO2 fallback/PGO.
3. **Source artifact**: paired patch + pose + PCD + provenance/manifest,
   retaining the authoritative existing validators and hashes.
4. **Optional independent processors**: manual PCD→PGM review and offline
   relocalization studies. Spatial confidence/terrain are research components;
   dynamic 3D cleaning and route authoring are not production-integrated.
5. **MapStore and publication**: map/reference path is a hint, NOT a publish
   command. Reuse existing Site 1.0 / READY Route and reviewed publisher.
   Runtime V4 owns active map, TF, relocalization, motion guard and tasks.

## Usage (offline)

    ./scripts/run_mid360_mapping.sh --capabilities
    ./scripts/run_mid360_mapping.sh /path/to/bag --dry-run \
      --product-config bringup/agt_mapping_bringup/config/product_pipeline.yaml

The original invocation works unchanged. Without an explicit --product-config,
an installed product_pipeline.yaml is used when present; older overlays without
that file retain the legacy backend_selection.yaml behavior.

For a second sensor, introduce a verified input adapter that specifies its
point message, IMU message, time units, frame semantics, driver ownership,
deskew assumptions, and rosbag preflight. Only after tests should it be
registered as a sensor input. A new sensor adapter does not require changing
MapStudio source artifact consumers.

For a second SLAM frontend, add an explicit backend_registry profile and
specific session launch/export implementation, run bag/source-package tests,
and then promote it to VERIFIED_SESSION_FRONTENDS. Simply changing YAML will
fail closed. Sensor input != LIO backend != offline processing backend.

## Optional processing backend semantics

Each configured optional processor is **manual** or **disabled**. 'manual'
records the separate existing entrypoint and never starts it during LIO
acquisition. There is no automatic shell/eval, implicit publication, or
post-export reprocessing. Experimental/unsupported processors fail preflight
when set to manual, with concrete capability blockers.

This avoids coupling heavy offline jobs to the persistent MID360/IMU runtime.
Later stages should use the existing source map and typed evidence contracts,
with bounded/cancelable jobs and separate authoring revisions.

The selected MapStore root is only a location reference. Before any actual
copy/publish, the existing strict validator/publisher remains authoritative.
Use development previews for incomplete local maps; formal acceptance must
not be silently weakened by a convenience profile.

## Minimum acceptance after sync to ROS2 Humble host

    PYTHONPATH=bringup/agt_mapping_bringup \
      python3 -m pytest -q bringup/agt_mapping_bringup/test/test_product_pipeline.py
    PYTHONPATH=benchmarks/agt_map_localization_benchmark \
      python3 -m pytest -q benchmarks/agt_map_localization_benchmark/test/test_study_workflow.py
    ./scripts/run_mid360_mapping.sh --capabilities
    ./scripts/run_mid360_mapping.sh /path/to/green-house --dry-run \
      --product-config bringup/agt_mapping_bringup/config/product_pipeline.yaml

Then run real-bag, MapStudio GUI and field validation separately. Neither
headless tests nor these docs count as real-world algorithm/route acceptance.
