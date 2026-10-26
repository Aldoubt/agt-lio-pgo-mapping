# Greenhouse problem-existence hypothesis matrix

| Hypothesis | Status | Evidence | Boundary |
|---|---|---|---|
| H1: Repeated row-middle causes stronger GLOBAL ambiguity. | NOT_TESTED | Row-middle final nominal GLOBAL success was 1/12 for FAST-LIVO2 and 0/12 for Point-LIO; headland was 1/12 and 0/12 respectively. The available trajectory-group proxy had 4/11 row-middle mismatches versus 9/10 headland mismatches. | Full descriptor Top-K scores, candidate spatial spread, normalized margins, and physical row labels were unavailable. The proxy is not physical wrong-row ground truth and does not show the hypothesized ordering. |
| H2: Headland has a larger local GICP basin. | SUPPORTED | Nominal headland-minus-row-middle basin difference was +21.8 pp for Point-LIO (scene-bootstrap 95% CI +5.9 to +37.7 pp) and +40.6 pp for FAST-LIVO2 (+25.9 to +53.4 pp). | This supports the local-basin proxy against same-session references, not physical observability or GLOBAL recovery. |
| H3: Multi-frame accumulation improves observability. | PARTIALLY_SUPPORTED | FAST-LIVO2 row-middle LOCAL success rose 37.8%→58.0% from 1→5 frames (104 improved, 3 degraded pairs); Point-LIO rose 34.0%→37.6% (44 improved, 26 degraded). FAST-LIVO2 had three sparse GLOBAL gains at five frames; Point-LIO GLOBAL outcomes were unchanged. | Headland LOCAL behavior differed by backend; GLOBAL success remains only 3/48 per backend across all frame counts. |
| H4: Scene effect persists across mapping backends. | SUPPORTED | Both repeatable references showed the same nominal ordering: HEADLAND > ROW_END > ROW_ENTRY > ROW_MIDDLE. | Compare effect direction only; frontend formats/frames differ and absolute map coordinates are not compared. |
| H5: Headland-triggered GLOBAL relocalization is reliable enough for deployment. | NOT_SUPPORTED | Across frames 1/3/5, headland GLOBAL success was 0/12 for Point-LIO and 1/12 for FAST-LIVO2; total GLOBAL success was 6/96. | This rejects reliability of the tested frozen BBS→GICP pipeline on this bag, not the general event-triggered architecture. |

## Overall problem-definition evidence

SUPPORTED as a problem-existence motivation: the local-basin scene effect is stable and replicated across two repeatable mapping references, while the tested GLOBAL pipeline is unreliable. This does not validate a proposed method or establish deployment reliability. H1 remains untested.
