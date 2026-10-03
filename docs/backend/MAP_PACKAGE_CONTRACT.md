# Backend-neutral frontend map package

All selected backends produce the same `map_package/` layout from normalized
frontend pairs:

```text
map_package/
├── map.pcd
├── poses.txt
├── poses_timed.txt
├── patches/0.pcd ...
├── calibration.yaml
├── metadata.yaml
├── manifest.yaml
└── checksums.sha256
```

`patches/i.pcd` is the local body cloud captured in the same synchronized pair
as pose row `i`. `map.pcd` is constructed by applying that row's
`T_frontend_body` to that exact patch. The validator checks every transformed
point and intensity to 5 μm, strict timestamp ordering, exact patch/pose
coverage, safe paths, backend identity, and checksums.

`metadata.yaml` records `mapping_backend.id`, project/mode, source revision,
configuration hash, explicit loop/GPS/external-correction flags, input bag and
topics, frame names, extrinsic, keyframe/point counts, and
`reference.absolute_ground_truth: false`. These are unoptimized same-session
frontend references. They are suitable as validated BBS/GICP source packages,
but are not promoted by the existing Nav map release path, which continues to
require its separately validated optimized-PGO artifact.

The greenhouse benchmark's `MapPackage` loader consumes only this common
schema, checksums, patches and timed poses. It does not select behavior based
on the LIO backend ID.
