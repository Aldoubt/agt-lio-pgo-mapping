# LIO backend selection evidence

## Current greenhouse acceptance

All current runs used `/home/yangxuan/rosbags/green-house`, playback rate 1.0,
ROS domain 126, no CPU affinity, and the same host. CPU and RSS below are sampled
for the complete launch process tree (bag player, estimator, adapter and map
exporter), not the estimator alone. The latency fields are frontend adapter
processing time and ROS-time output age; the estimators do not expose
per-scan processing latency or accepted/drop counters.

| Backend | Greenhouse map / route | CPU P50/P95 (one core %) | RSS P50/P95 (MiB) | Adapter latency P50/P95 (ms) | Repeatability | Status |
|---|---|---:|---:|---:|---|---|
| LIO-SAM no-loop | 959/934 keyframes; parallel rows visible; artifact validator PASS twice | 190.2/248.9 | 760/976 | 52.3/73.2 | **FAIL**: XY median 1.12 m, P95 3.68 m, max 3.98 m; yaw P95 10.0° | Not eligible for default |
| Point-LIO | 1,180 keyframes; row topology visible; artifact validator PASS | 48.5/57.4 | 497/574 | 0.33/1.04 | `NOT_RUN` (one current-bag run) | Supported candidate; no default claim |
| FAST-LIVO2 LIO-only | 1,162 keyframes; row topology visible; artifact validator PASS | 216.8/316.7 | 768/1,064 | 23.9/41.7 | `NOT_RUN` (one current-bag run) | Reference candidate; no default claim |
| FAST-LIO2 | Current deterministic replay audit found meter-scale spread across queue groups | `NOT_MEASURED` here | `NOT_MEASURED` here | `NOT_MEASURED` here | **FAIL** in the separate queue replay audit | `LEGACY / EXPERIMENTAL` |

LIO-SAM's first XY error exceeded 0.1 m at 1.75 s and 1 m at 33.25 s. Its
0.5 m XY occupancy IoU was 0.734 across the two runs. Keyframe count differed
by 25 (2.6%). Both exports were internally consistent, so the failure is in
frontend replay repeatability, not package self-consistency. Full metrics and
run paths are in
[MULTI_LIO_BACKEND_ACCEPTANCE.md](MULTI_LIO_BACKEND_ACCEPTANCE.md).

The three single-run maps show the same greenhouse row topology in the common
XY view, but this is a visual topology check rather than an absolute-ground-
truth accuracy result. Point-LIO and FAST-LIVO2 have not been replayed a second
time. Their default eligibility is therefore unestablished.

## Historical benchmark evidence

| Backend | Historical map quality | CPU | RSS | Latency | Repeatability | Evidence status |
|---|---|---:|---:|---|---|---|
| LIO-SAM no-loop | `NOT_MEASURED` | 88.72% mean | 488.99 MiB peak | `NOT_MEASURED` | `NOT_MEASURED` | One older run, different bag |
| Point-LIO | Trajectory-health check failed with extreme path divergence | 29.29% mean | 446.63 MiB peak / 207.03 MiB mean | `NOT_MEASURED` | `NOT_MEASURED` | Historical failed run |
| FAST-LIVO2 LIO-only | Historical visualization only; no validated map artifact | 124.29% mean | 976.18 MiB peak | `NOT_MEASURED` | `NOT_MEASURED` | One older run, different bag |
| FAST-LIO2 | Not included in that benchmark | `NOT_MEASURED` | `NOT_MEASURED` | `NOT_MEASURED` | See separate queue replay audit | No old conclusion inferred |

Historical source/configuration details are in
[LIO_BACKEND_AUDIT.md](LIO_BACKEND_AUDIT.md). The older runs do not rank the
current greenhouse results: they use another bag and do not provide absolute
ground truth or repeatability.

## Default selection

**DEFAULT: `DEFAULT_NOT_ESTABLISHED`**

LIO-SAM no-loop was tested twice with the pinned six-axis compatibility patch,
explicit loop closure off, GPS off and external global correction off. Its
replay repeatability failed by meter-scale XY and multi-degree yaw differences.
The other two frontend maps pass the package and visual row-topology checks,
but each has only one current-bag run and no absolute ground truth. None meets
the default evidence gate. FAST-LIO2 remains legacy/experimental.
