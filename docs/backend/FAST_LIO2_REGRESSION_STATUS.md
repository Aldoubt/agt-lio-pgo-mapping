# FAST-LIO2 regression status

Status: **LEGACY / EXPERIMENTAL / NOT ACCEPTED AS A MAPPING REFERENCE**

The local source at `/home/yangxuan/ros2_ws/src/external/fast_lio2_mapping`
is `Aldoubt/fast-lio2` HEAD
`a12f8db1718ef23075b24817bca927cab9cca021`; the framework `.repos` file pins
its parent `7f664a66ae7b3b683dddf7f5322c1ea55a3c8141`. The local repository has
a pre-existing `lidar_processor.cpp` modification. Neither source nor formal
install was overwritten in the queue replay audit.

The report and full captures are at:

`/home/yangxuan/ros2_ws/experiments/fastlio2_queue_replay_20261002/`

Evidence on `/home/yangxuan/rosbags/green-house`:

- Q1 (LiDAR depth 1, IMU depth 400) dropped LiDAR callback messages and had
  up to 2.793 m pairwise XY spread and 3.187 degrees yaw spread.
- Q10 (LiDAR depth 10, IMU depth 400) received all 6,230 LiDAR callbacks and
  processed identical sensor-stamp sequences across repeats, but still had up
  to 1.346 m XY and 2.559 degrees yaw pairwise trajectory spread.
- ORIGINAL (LiDAR depth 10, IMU depth 10) lost IMU callbacks and had up to
  6.122 m XY and 6.193 degrees yaw pairwise spread.
- The Q10 raw-direct vs adapter comparison also differs, but Q10 raw-direct is
  itself non-repeatable, so the whole trajectory spread cannot be assigned to
  the adapter alone.

Conclusion: queue depth 1 causes additional LiDAR loss and larger observed
spread in this run matrix, but depth 10 did not restore deterministic replay.
The current measurements do not isolate `a12f8db` as the cause. It is therefore
not labelled a suspected regression and no rollback is recommended from this
evidence alone.

The first measured trajectory divergence appears in FAST-LIO2 state/map
processing after the same Q10 processed sensor timestamps. This is evidence of
non-repeatability, not a diagnosis of the internal root cause. The experiment
does not prove physical trajectory correctness or compare against absolute
ground truth. PGO/BBS/GICP were not part of this regression run.

To return the repository integration to the pre-feature behavior, revert only
the feature commits on the feature branch. Do not reset the external FAST-LIO2
checkout, discard its `lidar_processor.cpp` change, or change the formal install
as a rollback shortcut. Any source-pin change must be a separately reviewed
manifest/config change with a fresh isolated build and replay report.
