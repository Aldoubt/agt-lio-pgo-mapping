# Mapping product integration audit — 2026-10-09

## Exact starting point

Remote repository: Aldoubt/agt-lio-pgo-mapping.
Integration branch: integration/mapstudio-mapping-product-v1.
Base ref: feature/greenhouse-topology-topk-v1 at
4e993c5d33fc7cc110a578103a6b5576a181ca06.
Operator source: feature/mapstudio-operator-workflow-v1 at
111db625ab5bd8f3f6caa347ba37800677292e24.
YHS source: feature/yhs-field-appliance-v1 at
539352d309deef4f0acfe5f2bd783d5e72623a88.

The topology/Operator branches diverge by one tip commit each (both are
46 commits ahead of the historical main). Never merge them by file overwrite.

## Round 1: files safely migrated (bounded subset)

Ported unchanged from Operator Workflow:
- offline study_workflow.py (Query Set, immutable Study and job report)
- test_study_workflow.py
- query-set/study JSON schemas
- docs/contracts/mapstudio_operator_workflow_v1.md and acceptance record
- isolated PointCloudRenderSampler.hpp and its test source

Registered agt_mapstudio_workflow in the benchmark console_scripts.

The current Research Asset Contract V1, Annotation editor, aisle-proposal
generator and UX cleanup from the topology branch are preserved. Porting
the other divergent C++ GUI MainWindow/PointCloudViewer implementation is
explicitly deferred until a single combined UI change and GUI regression
can be reviewed; unconnected C++ helpers are NOT claimed as integrated GUI.

The original Operator acceptance marked software PASS in its source branch
and physical-display acceptance NOT VERIFIED. That historical claim is not
a test result for this integration branch.

## Round 2: new code and integration review

New product_pipeline.py:
- explicit, typed MID360 sensor input contract
- reuse of backend_registry profiles; only FAST-LIVO2 LIO-only is an
  accepted automatic mapping session, no automatic fallback
- optional processor catalog (manual/research/proposed)
- YAML validation, conflict rejection and truthful capability report
- no ROS import, device launch, TF publication or runtime active-map writes

Existing mapping CLI now accepts --product-config and --capabilities.
Standalone product test cases cover bad modes, unknown sensors/backends,
misconfigured topics, data preservation and no fake automatic stages.
The new config is installed by the existing config/*.yaml setup rule.

## Matrix (not equivalent to an acceptance percentage)

| Capability | Status on integration branch | Next acceptance |
| --- | --- | --- |
| Verified MID360/FAST-LIVO2 source session | Existing implementation | Replay complete green-house bag |
| Replaceable LIO backend registration | PARTIAL: catalog/profiles exist, only one session accepted | Implement separate session/export adapter per algorithm |
| Sensor swap | PARTIAL: typed boundary, MID360 only | New sensor input and timing acceptance |
| PCD/patch/pose provenance | Existing | Run canonical artifact validator on real output |
| 2D map edit/review | Existing manual tool | Edited map/export consistency A1 |
| Dynamic 3D cleanup | PROPOSED | Static evidence + ray visibility, no silent PCD rewrite |
| Terrain/traversability | Research | Producer/consumer schema and held-out terrain review |
| Top-K / GICP offline Study | Ported CLI; integration unverified | Study regression and GUI evidence review |
| Row/structure and cross-stage annotations | Existing topology branch | Human physical row IDs/review/freeze |
| MapStudio combined Operator GUI | NOT INTEGRATED | Resolve C++ divergence and real Qt GUI check |
| Route authoring | PROPOSED | READY Route compatibility with V4 |
| Strict Site/Route publication | Existing partial | A0/A1/A3 production gates |
| Shared map folder / active map | Reference only | V4 owns active map; compatible loader contract |

## Explicitly unmodified

- main and both historical feature branches
- the pinned SLAM source, BBS/GICP, active map, navigation runtime and robot TF
- mapping source data, existing published maps and Study evidence
- Site / Route / Robot Profile schema and authoritative checksum rules

## Tests at authoring time

Code committed remotely. No current ROS 2 Humble, Qt/OpenGL, green-house
rosbag or executable GitHub workspace was available to this authoring
session. All new unit tests and integration/GUI/field checks are **NOT_RUN**,
not PASS. Reviewers must execute the commands in product_mapping_architecture_v1.md.

## Follow-up actions

P0: run pure Python regression, static lint and source import checks.
P1: reconcile the divergent MapStudio GUI in an isolated commit.
P1: verify edited PCD/patch/index invalidation and 2D export (A1).
P1: test normal/raw-bag/live entrypoint and true cancellation behavior.
P2: stage an independent sensor adapter or alternative LIO frontend, never
change the verified operator default without acceptance.
P2: dynamic 3D cleaning + confidence/terrain after source/asset consistency.
