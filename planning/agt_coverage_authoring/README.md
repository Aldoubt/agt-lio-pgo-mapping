# agt_coverage_authoring

Offline greenhouse route authoring: reviewed structure + Nav2 map + existing
Ackermann profile → dual-mode coverage, clearance and feasibility reports.
No TF, velocity commands, map activation or task execution.

See [operator and integration guide](../../docs/coverage_authoring.md).

```bash
python3 -m pip install -e 'planning/agt_coverage_authoring[reference,test]'
agt_coverage_authoring plan --request planning/agt_coverage_authoring/examples/rows.yaml --output /tmp/coverage-rows
agt_coverage_authoring plan --request planning/agt_coverage_authoring/examples/area.yaml --output /tmp/coverage-area
```

The examples are synthetic and explicitly select the `reference` backend.
This backend enumerates pinned `rsplan` analytic Reeds–Shepp candidates, with
Shapely polygon clipping for swaths. It is not Smac obstacle search or proof
of route nonexistence. `ros` selects the isolated OpenNav/Smac backend;
missing dependencies fail instead of silently falling back.

`clearance.npz` records a conservative unsigned distance in metres, known/free
masks, grid origin and resolution. Unknown space is never free. Whole-vehicle
swept validation remains authoritative for offline geometric feasibility.

`plan.json`/`report.json` retain directions, gear-change events, task semantics,
missing targets and bound input file hashes. `export --format draft` uses the
existing Route CSV columns and a DRAFT manifest. READY requires the existing
V4 Site/Profile/Route owners, exact identity bindings, verified limits and a
compatible complete route. It does not enable motion.
