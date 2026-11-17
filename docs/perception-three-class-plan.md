# P5-02 three-class training plan

## Active inference classes

The active model predicts three pixel classes: **regolith**, **rock**, and
**shadow**. To retain usable rocky-surface annotations, bedrock ID 1 and
discrete-rock ID 2 are both mapped to output class 1 (`rock`). Crater ID 3 is
outside this model's scope and maps to ignore ID 255. Existing ignore pixels
remain ignored. This mapping does not claim that craters are safe; crater risk
must continue to come from the geometry layer until a crater model is qualified.

The reviewed five-class source dataset remains immutable. The derivation writes
a separate mask for every row in the 2026-11-10 canonical manifest, preserving
the existing source, scene, and train/validation/test assignments. Training
images are reused by reference. `tools/build_three_class_dataset.py` records
the input manifest hash, row counts, pixel counts, and full mapping in its build
report. Derived masks and training checkpoints are stored under `.build/`.

## Qualification

The scene-isolated validation and held-out test splits remain unchanged. A new
three-class gate is frozen in `config/perception/gates-three-class.json` before
training: mIoU >= 0.50 and each class IoU >= 0.25. These thresholds match the
prior engineering bar; historical five-class scores are not relabeled as
three-class results. P5-02 must train a fresh three-output model, select its
checkpoint on validation, and evaluate the held-out test split once. Report
class support, class IoU, mIoU, confusion matrix, data/config/checkpoint hashes,
and whether the new gate passed.

The original five-class ontology and artifacts remain archived as source-data
provenance. Active inference and ROS integration use only the qualified
three-class package. A failed gate would leave the model unqualified for
integration.

## Qualification result (2026-11-12)

The trained EMA export in `all_outputs.zip` passed the frozen three-class gate
on both validation and held-out test data. Validation mIoU was 0.5926; held-out
mIoU was 0.6057 with IoU 0.9181 for regolith, 0.3691 for rock, and 0.5300 for
shadow. The minimums remain mIoU >= 0.50 and per-class IoU >= 0.25. The
checkpoint SHA-256 is
`16276d1375f023d04d3f60557a3c6a84f3d5dbfc97c51d4aea3deeec8c5a8210`.

The held-out evaluation used the unchanged 749-row scene-isolated test split
once and the export's recorded preprocessing. Detailed evidence and the
reproducible evaluator are in
`evidence/runs/20261112-p502-threeclass/P5-02/`. P5-02 is complete for the
three-class engineering scope. P5-03 export parity and package inference checks
pass; the ROS pipeline now consumes this model and preserves its confidence
output. The historical five-class model remains archived and unqualified. The
three-class score is limited to this sparse lunar-analog corpus.
