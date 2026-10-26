# Top-K metric definitions

- Physical-row and along-row recall are N/A for queries without a confirmed row; headlands normally have no row.
- Headland-region and XY-region recall are supplemental region metrics, not physical-row recall.
- UNKNOWN candidates are reported separately; they are not wrong rows or failed retrievals.
- WrongRowFraction uses the full prefix and is unavailable if labels are missing; known-row conditional fraction and bounds are supplemental.
- Row entropy normalizes over known physical rows only; known weight coverage is reported.
- Unattempted, timed-out invalid, or incompletely observed stage outcomes are censored, not failed.
- Native success is not runtime policy acceptance; FinalAcceptedSuccess remains N/A without runtime_policy evidence.
- K refers to descriptor prefix; stage@K uses only actual observed candidates in that prefix.
- A known hit makes Recall@K true; a miss is false only for a complete and fully observed prefix.
- Longitudinal summary rows average per-query candidate statistics, not independent candidate samples.
- Cross-backend effects pair scene IDs; explicit backend-to-canonical transforms are required for physical labels.
- A rigid same-timestamp alignment has residual error and does not establish physical ground truth; transformed labels require manual review against row spacing/corridors.
- Shared BBS time budgets mean stage outcomes in a prefix are not a counterfactual new run with a smaller candidate_top_k.

Descriptor similarity is higher-is-better; ring distance is lower-is-better. M12 = (similarity1 − similarity2) / (|similarity1| + 1e−12); ring distance is never mixed into M12. Entropy defaults to uniform rank-frequency over known row labels. Softmax requires an explicit fixed tau. Δs is compared only within the same physical row. Pose-region uses strict |Δs| < 1/2/5 m; longitudinal ambiguity reports strict |Δs| > 1/2/5 m. XY-region uses strict distance < 1/2/5 m. BBS basin hit requires valid coarse XY ≤ 1 m and yaw ≤ 10° versus frontend reference; GICP requires converged final XY ≤ 0.5 m and yaw ≤ 5°. These are fixed reference-tolerance proxies.

Eight core CSV files use scene/query units. Additional query and candidate CSVs retain missing values, observed/censored counts, physical labels, and raw descriptor scores. Scene contrasts use independent scene bootstrap; multiframe/backend contrasts use scene-paired bootstrap. One scene per group has no CI.
