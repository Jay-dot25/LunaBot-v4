# P10-03 — Cross-trial report

The report builder combines the frozen P10 scenario trials with the accepted
P5 held-out perception, P5 inference profile, P7 replan benchmark, P8 live
round-trip acceptance, and P3 short-route localization comparison. It records
SHA-256 hashes for every input, renders a machine-readable report and Markdown
summary, and produces four SVG plots.

Run from the repository root:

```bash
python3 tools/build_evaluation_report.py \
  --output-dir evidence/runs/<run-id>/P10-03
python3 -m unittest tests/test_evaluation_report.py -v
```

`config/evaluation/report.schema.json` defines the report envelope. The builder
also checks that every metric in `config/evaluation/metrics.json` appears once,
every unavailable metric has a reason, and all source inputs exist and are
hashed. A report may be complete while carrying limitations; it does not turn
planner-fixture results into live mission acceptance.

The 2026-11-13 report is at
`evidence/runs/20261113-p1003/P10-03/report.md`. Its status is
`COMPLETE_WITH_LIMITATIONS`: it reports existing offline/fixture evidence and
marks travel-time, trajectory-length error, physical collision count, physical
clearance, and recovery measurements unavailable where their required sources
were not recorded. The other nine metrics are labeled limited to their evidence
scope.
Four non-roundtrip scenario types still need live mission trials for runtime
claims.
