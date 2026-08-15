# WCR clearance-safe trajectory rerun

Source run identifier: `20260812_165826_clearance_rerun`

This rerun uses two obstacle boundaries:

- Hard safety boundary: robot radius `0.18 m` + safety margin `0.02 m` = `0.20 m`.
- Planning boundary: hard boundary + tracking allowance `0.05 m` = `0.25 m`.

A run succeeds only when the planner reports completion, the final pose is settled, the planned polyline remains outside the planning boundary, and the continuously sampled actual trajectory remains outside the hard safety boundary.

## Results

| Metric | Mode 1 | Mode 2 |
| --- | ---: | ---: |
| Completed | yes | yes |
| Safety violation | no | no |
| Intrusion samples | 0 | 0 |
| Minimum actual clearance outside hard boundary | 42.782 mm | 36.323 mm |
| Minimum planned clearance outside hard boundary | 50.241 mm | 55.241 mm |
| Minimum planned clearance outside planning boundary | 0.241 mm | 5.241 mm |
| Final position error | 14.954 mm | 5.719 mm |
| Actual path length | 1.006188 m | 1.088883 m |

The minimum-clearance verification samples every recorded trajectory segment at no more than `1 mm` spacing. An independent post-run check was also performed at `0.5 mm` spacing.

## Files

- `combined_summary.json`: combined metrics for both modes.
- `mode1/` and `mode2/`: raw trajectory, obstacle, plan, status, simulator, and recorder logs.
- `figures/mode1_result.png` and `.svg`: mode 1 trajectory, motion, tracking, and clearance.
- `figures/mode2_result.png` and `.svg`: mode 2 trajectory, motion, tracking, and clearance.
- `figures/modes_comparison.png` and `.svg`: two-mode trajectory and metric comparison.
