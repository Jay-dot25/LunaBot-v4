# P5-01 perception dataset audit and annotation protocol

## Active P5-02 model scope

The active training/inference plan uses three classes: regolith, rock, and
shadow. Bedrock masks merge into rock; crater pixels become ignore (255), and
crater-risk reasoning remains in the geometry layer. The source five-class
ontology and historical runs remain intact. See
[`perception-three-class-plan.md`](perception-three-class-plan.md) for the
full-corpus derivation, preserved splits, and new qualification gates.

## Audit result

The repository contains no training images, annotation masks, split file, or model. P5-01 adds a rights/source audit, a five-class ontology, a scene-level split plan, and a validator. It does not claim that external images were downloaded or that the target masks have already been created.

Selected sources are in config/perception/sources.json:

- NASA POLAR supplies 12 lunar-analogue terrain experiments, stereo imagery, rocks, craters, and hard-shadow lighting. NASA's Science Data Portal says NASA-led mission data are CC0 unless an individual file carries a restrictive notice or license. Each acquired artifact must retain a file notice/metadata check.
- POLAR-Sim supplies related masks and terrain assets from Dryad under Dryad's CC0 data terms. Existing masks cover ground, rock, and rock shadow; they do not provide the final crater and bedrock annotations.
- The Mars-Bench MSL mirror declares CC-BY-4.0 and includes bedrock, gravel/sand/soil, rock, shadow, and other classes. It is restricted to bedrock/domain pretraining. Its images are Mars data, never held-out lunar/analog evaluation data. Verify attribution and upstream terms against the exact downloaded revision before ingestion.

LuSNAR is excluded because its repository's MIT claim conflicts with a commercial-use contact restriction, and its linked LICENSE currently returns 404. The NASA AI4Mars Zenodo record is excluded because its NASA catalog entry says no license is specified while the Zenodo record reports IEEE copyright. Do not infer reuse rights from public accessibility.

## Five-class mask ontology

Masks are indexed uint8 PNGs: 0 regolith, 1 bedrock, 2 rock, 3 crater, 4 shadow, and 255 ignore. Class definitions are in config/perception/classes.json.

- Regolith: loose or granular continuous surface. Use source-specific POLAR ground labels only after confirming the pixels depict granular ground.
- Bedrock: continuous exposed, consolidated substrate with visible structure. Do not infer it from dark color, a shadow, or a single large rock. Only the CC-BY bedrock source supports this class before new lunar/analog annotation.
- Rock: discrete positive-relief rock/cobble/boulder; cover the visible object, not its cast shadow.
- Crater: a negative-relief depression and inner rim when shape or registered terrain geometry supports the interpretation. Do not label arbitrary dark regions, pits without evidence, or rock shadows as craters.
- Shadow: cast shadow independent of substrate. Do not infer a shadow from dark albedo alone.

For single-label masks, shadow has precedence over underlying material. The crater label has precedence over regolith/bedrock in the defined crater region. Occluded pixels are labeled shadow only when the cast-shadow boundary is supported; ambiguous boundaries and unobservable pixels use 255. Keep RGB source images, masks, source IDs, scene IDs, timestamps, annotator/reviewer IDs, and license attribution linked.

## Leakage-safe split plan

config/perception/splits.json assigns complete POLAR terrain IDs, rather than frames, crops, exposures, or camera views, to one split:

- Train: terrain01-08
- Validation: terrain09 and terrain11
- Test: terrain10 and terrain12
- Exclude terrain13 from qualification because NASA describes it as extra and not well calibrated.
- Every derived crop, stereo view, exposure, annotation, or augmented copy inherits its parent scene split.
- Mars bedrock data may be training-only; it cannot contribute validation/test frames or support claims about lunar bedrock accuracy.

The terrain_id separation is validated with tools/validate_perception_manifest.py. Class frequencies and per-class support must be measured from acquired masks before P5-02 freezes training and evaluation thresholds. If a class is missing from a held-out split, report its metric as unavailable; do not move related frames across splits to force coverage.

## Commands

    python3 tools/validate_perception_manifest.py --classes config/perception/classes.json --sources config/perception/sources.json --splits config/perception/splits.json
    python3 -m unittest tests.test_perception_manifest -v

## Rights and domain sources

- NASA POLAR dataset: https://ti.arc.nasa.gov/dataset/IRG_PolarDB/
- NASA science data license policy: https://science.data.nasa.gov/about/license
- POLAR-Sim (Dryad DOI 10.5061/dryad.ksn02v7hf): https://doi.org/10.5061/dryad.ksn02v7hf
- Dryad reuse terms: https://dev.datadryad.org/help/guides/reuse
- Mars segmentation mirror and its CC-BY-4.0 dataset card: https://huggingface.co/datasets/gremlin97/mars-seg_msl

The external dataset is not copied into this repository. The materialization state is explicit in config/perception/splits.json.


## P5-02 training and evaluation implementation

`tools/train_perception.py` consumes a reviewed CSV with columns `image,mask,source_id,terrain_id,split`. Paths may be absolute or relative to the CSV directory. The loader checks approved source IDs, the exact terrain-to-split policy, training-only source restrictions, and file existence before creating a model. Polar-Sim scene IDs must follow the split table; Mars segmentation records may be included in `train` only. Every mask must be indexed grayscale/palette PNG using IDs 0–4 and ignore 255.

The trainer uses a small deterministic U-Net, seeded Python/NumPy/PyTorch RNGs, train-only optimization, validation mIoU checkpoint selection, and exactly one final held-out test evaluation. A required `--gates` JSON must contain `schema_version: 1`, a unique `gate_id`, `minimum_miou` in [0,1], and `minimum_per_class_iou` thresholds for exactly all five canonical classes. The file hash and thresholds are recorded; the test pass fails if any class lacks support or misses its threshold. This prevents test use before numeric gates are frozen. It writes a checkpoint and `metrics.json` containing software/device/configuration, split and manifest hashes, per-file hashes, record counts, class support, per-class IoU, mIoU and confusion (rows=true, columns=predicted). Unsupported classes report null IoU and are counted explicitly; they cannot be interpreted as passing a class score gate. Use image size 256 and batch size 2 by default for the 4 GB laptop GPU; CPU is supported with `--device cpu`.

Example after reviewed data is materialized outside Git:

```sh
python3 tools/train_perception.py --manifest /path/to/reviewed-manifest.csv --gates /path/to/frozen-gates.json --output /path/to/new-run-directory --epochs 30 --batch-size 2 --image-size 256 --seed 20261104 --device auto
```

Metric unit and split-manifest verification:

```sh
python3 -m unittest tests.test_perception_training tests.test_perception_manifest -v
python3 -m py_compile tools/train_perception.py tools/perception_metrics.py
```

### Qualification status (2026-11-04)

Historical qualification status as of 2026-11-04: implementation fixtures and a synthetic-only CPU smoke passed, but annotation archives had not yet been downloaded and real training, held-out metrics, and frozen numeric gates did not exist. The attempted Dryad file-stream/API retrieval was blocked at that time. The current acquisition and materialization state is recorded below; it supersedes the earlier data-availability statements. No real held-out metrics are claimed.


## POLAR-Sim archive materialization

`tools/materialize_perception.py` pairs Dryad YOLO polygons with NASA PNGs by terrain, camera position, illuminator, sun azimuth, side, and exposure. It writes canonical indexed masks: source ground class 1→regolith 0, source rock class 2→rock 2, source rock-shadow class 3→shadow 4; source background and unlabeled pixels become ignore 255. Bedrock and crater are deliberately not inferred from background, terrain names, or color. Original archives and derived images stay outside Git.

Run with complete archives:

```sh
python3 tools/materialize_perception.py --labels-dir /tmp/lunabot-p502/polar-sim-label-archives --nasa-dir /tmp/lunabot-p502/nasa-polar-archives --output /tmp/lunabot-p502/materialized-<run-id>
```

For diagnostic partial conversion, add `--allow-partial`. The tool reports missing archives, unmatched/ambiguous labels, per-terrain and per-split counts, pixel support, and embedded NASA notice candidates. It does not report training-ready if a split or target class lacks support.

### Current staged-data audit (2026-11-05)

All twelve POLAR-Sim label ZIPs and all twelve corresponding NASA Terrain01–Terrain12 ZIPs are staged outside the repository under /home/jay/lunabot-p502-persistent/; every archive passed ZIP integrity checks. Materialization at /home/jay/lunabot-p502-persistent/materialized-20261105-002022 produced 3,879 records: train 2,319, validation 803, test 757. All 3,879 images have masks, all 3,879 masks have images, and image/mask dimensions match. Pixel support is regolith 0: 3,808,319,983; rock 2: 200,897,482; shadow 4: 224,681,292; ignore 255: 4,897,949,947. Bedrock 1 and crater 3 remain absent; the report correctly returns PARTIAL_DATA_ONLY.

### LIDAR-derived crater candidate workflow

NASA Terrain05, Terrain11 and Terrain12 archives include GroundTruth_ascii PosA.xyz, PosB.xyz and PosC.xyz plus stereo.calibration. Each XYZ row contains LIDAR raster row/column, XYZ meters and intensity. NASA documents transforming LIDAR points into the left-camera frame, projecting with the camera matrix, and applying lens distortion. A sampled Terrain11 PosA check verified the inverse CAM_LIDAR transform: 349,998 of 411,672 sampled points landed inside the 1936x1216 frame; the generated overlay visually aligned with the terrain.

tools/project_crater_candidates.py accepts explicitly supplied polygons in the LIDAR XY frame. It writes sparse class-3 candidate point masks and a bounded set of overlays in a separate output directory. A projected point becomes crater only where the paired source mask already says regolith (0); known rock, bedrock, shadow and unknown pixels remain unlabelled. Unprojected pixels stay ignore 255. It does not edit source masks or the materializer's manifest, and its report always sets training_ready=false. Human review and coverage/occlusion checks are mandatory before candidates can be merged into a reviewed training dataset. The 2026-11-06 regenerated run has 131 masks; Terrain12 has substantial known non-regolith exclusions, while Terrain11 has none in its existing labels despite visible boulders. That gap confirms the overlays are proposals, not accepted ground truth.

Annotation JSON example (illustrative coordinates only):

    {"schema_version":1,"rois":[
      {"terrain_id":"terrain11","position":"A","annotator_id":"your-id",
       "polygon_xy_m":[[-5.0,-1.0],[-4.0,-1.0],[-4.0,1.0]]}
    ]}

Candidate command after creating reviewed ROI polygons:

    python3 tools/project_crater_candidates.py --manifest /home/jay/lunabot-p502-persistent/materialized-20261105-002022/manifest.csv --nasa-dir /home/jay/lunabot-p502-persistent/nasa-polar-archives --annotations /path/to/crater-rois.json --output /path/to/new-crater-candidate-run

Do not use the example coordinates as an annotation. The current state has no accepted crater ROI JSON and no class-3 training masks. A separate pinned MarsSeg MSL train-only shard is now acquired and materialized with bedrock support; it does not supply lunar/analog bedrock evaluation support. No crater ROI JSON has been reviewed. No model training or real held-out metrics are claimed.


## MarsSeg MSL bedrock pretraining source (2026-11-05)

The approved mirror gremlin97/mars-seg_msl was pinned to revision 35b03e48bcf9ab2e26980ccb58e4419d290e06c5. Only the train shard was downloaded. The source card declares CC-BY-4.0 and leaves its citation as TBD. Exact revision, README, mapping, and parquet SHA-256 are recorded outside Git in /home/jay/lunabot-p502-persistent/mars-seg-msl/35b03e48bcf9ab2e26980ccb58e4419d290e06c5/source-manifest.json. Preserve credit to the dataset uploader and AI4Mars/Mars-Bench provenance; confirm any upstream citation terms before redistribution.

The reproducible converter tools/materialize_marsseg_train.py writes outside the repository to /home/jay/lunabot-p502-persistent/materialized-marsseg-train-20261105/. Mapping: source bedrock 1 to canonical 1; gravel/sand/soil 2 to regolith 0; rock 3 to rock 2; shadow 4 to shadow 4; background 0, sky/mountains 5 and track 6 to ignore 255. It emits 2,893 paired RGB/mask PNGs and a train-only CSV plus a hash/count report. Full audit found zero pair mismatches, bedrock support 266,581,689 pixels, and no crater pixels. The project trainer accepted a combined manifest with 5,212 train, 803 validation and 757 test rows; all 2,893 Mars rows stayed in training.

Run using the designated AI environment, which requires pyarrow and Pillow:

    /home/jay/lunabot-ai/venv/bin/python tools/materialize_marsseg_train.py --parquet /home/jay/lunabot-p502-persistent/mars-seg-msl/<revision>/train-00000-of-00001.parquet --mapping /home/jay/lunabot-p502-persistent/mars-seg-msl/<revision>/mapping.json --source-readme /home/jay/lunabot-p502-persistent/mars-seg-msl/<revision>/README.md --revision <revision> --output /path/to/new-output
    /home/jay/lunabot-ai/venv/bin/python -m unittest tests.test_materialize_marsseg_train -v

This is cross-domain bedrock pretraining data only. It does not count toward held-out lunar/analog support, has no crater labels, and does not make the five-class data training-ready. Do not train or report qualification metrics until reviewed crater labels/support exist across the required splits and numeric gates are frozen. The trainer also performs a mask-pixel support preflight on all three splits and exits before model creation if any canonical class is absent; its fixture confirms class 255 remains ignored.


### Crater boundary review packet (2026-11-05)

An external review packet is available at /home/jay/lunabot-p502-persistent/crater-review-packet-20261105/. Open tools/annotate_crater_rois.html from the repository in a browser to click crater boundaries on the PosA XY elevation maps. It converts clicks into LIDAR XY coordinates and exports schema-compatible JSON; enter an annotator ID, add one polygon per crater, review the generated JSON, then provide crater-rois.json for mask projection. The packet contains Terrain05 (train), Terrain11 (validation), and Terrain12 (test) aids plus review-manifest.json with source archive and image hashes. The camera overlays show registration, not crater pixels. The offline annotator was blocked in the Codex preview; a provisional JSON was instead created from visible Terrain11/12 contours with annotator_id codex-proposal-unreviewed. Terrain05 was omitted because the visible black gaps are missing LIDAR returns. Projection created 131 separate review-only candidate masks; no candidate is accepted as a training label. Codex inspection found that the proposed basins align with the crater bowls, but visible boulders remain inside at least one candidate because the corresponding source mask does not label them as rocks. Terrain05 camera imagery and missing-return regions do not give a sufficiently supported boundary for us to author a train ROI from those dark patches. Keep the draft polygons unaccepted; a train-split crater label source and reviewed held-out bedrock labels remain necessary. The proposal, regenerated report, and review overlays remain outside Git.

### P5-02 resumed audit (2026-11-06)

This update supersedes the earlier statement that no train crater labels exist. The delegated Terrain05 PosA annotations were merged into 68 separate training masks, adding 850,772 crater pixels while retaining known rock/shadow/unknown pixels. All validation/test masks remain unchanged; the 131 held-out crater candidates remain proposals. The merge tool now rebases image and unchanged-mask paths to absolute paths so the relocated output manifest resolves correctly. The earlier external merged CSV had broken relative references and should not be used. The regenerated manifest is at evidence/runs/20261106-p502/P5-02/rebased-merge/manifest.csv (local derived data, ignored by Git).

All 29 perception materializer/projector/merger/trainer/manifest fixtures pass, including the relative-path regression. Compilation and git diff --check pass. The reproducible audit and measured counts are in evidence/runs/20261106-p502/P5-02/audit_continuation.py and continuation-audit.json. Five-class qualification remains open: validation/test have no accepted bedrock or crater support. Do not invent bedrock from discrete rocks or move Mars training frames into held-out splits. No qualification training, frozen gates, or held-out score is claimed.

### P5-02 annotation repair retry (2026-11-06)

Full-resolution review produced two conservative partial crater annotations: Terrain11 PosA `11_A_off_180_L_0032.png` has 551 geometry-supported reviewed crater pixels; Terrain12 PosA `12_A_off_180_L_0032.png` has 2,201. The corresponding unreviewed crater rectangles are ignore, preventing known ambiguous rock/shadow regions from being promoted to crater. The review applies only to those exact frames and hashes; other illumination/exposure variants remain unaccepted. These small patches are partial annotations and do not establish representative crater accuracy. The new manifest is a diagnostic retry corpus, not a completed qualification corpus. Review definitions, hashes and limitations are in `evidence/runs/20261106-p502/P5-02/heldout-patch-review.json`; reproducible materialization and retry scripts accompany it.

Bedrock remains a data-acquisition dependency. Reinspection of the staged held-out scenes did not establish continuous exposed consolidated substrate. An external-source search found the Concordia/Mission Control Lunar Analogue publication with exposed-bedrock examples, but no accessible dataset download in its linked public code repository; the user has no copy. MADMAX requires registration and describes academic use; Devon Island did not establish a compatible licensed bedrock mask corpus. No alternative source was silently approved. See `bedrock-source-search.json` for exact references and findings. No bedrock labels were fabricated and no qualification training was run. The retried preflight result is retained separately in `retry-preflight.json`.


### P5-02 independent corpus review (2026-11-07; supersedes readiness claims)

The inherited Terrain05 crater merge is quarantined pending correction. Inspection of full-resolution 05_A_off_180_L_0032.png and the 12-frame training montage shows accepted regions overlapping prominent positive rocks, with masks propagated through saturated and dark exposures. Successful geometric projection and unit tests do not establish semantic correctness. The restarted CPU run was stopped before held-out testing; its outputs must not be used as qualification evidence. NASA documents small negative features in Terrain05, but that does not validate these particular regions.

The frozen 256px cache also contains 276 identical completely white RGB frames across train (161), validation (59), and test (56). Remove these uninformative frames from all splits rather than relocating them. A first audit additionally keyed lineage by image content and therefore conflated same-image/different-mask entries; that auditor bug is corrected to use the cache key derived from the original path. The original audit report is retained for traceability and will be superseded by the corrected report.

The previously acquired Apollo and BASEPROD additions provide sparse bedrock evaluation support; claims that bedrock is wholly absent are stale for the combined corpus. They remain limited in domain, scale and scene count. A separate NASA POLAR Traverse training-only crater acquisition is being prepared; existing held-out Stereo terrain assignments are preserved.

### P5-02 final training and qualification result (2026-11-08)

The reviewed corpus contains 5,417 train, 746 validation and 749 test rows. All five classes are supported in each split, and no identical RGB image crosses splits. Frozen gates are mIoU >=0.50 and every class IoU >=0.25. Thirty CUDA epochs completed; validation selected epoch 23 at mIoU 0.302. Held-out test mIoU is 0.310 (regolith 0.898, bedrock 0.001, rock 0.228, crater 0.003, shadow 0.420). Qualification FAILED. The model is not suitable for integration. Sparse bedrock/crater annotations and mixed-domain coverage remain limiting. Full provenance and confusion matrices: `evidence/runs/20261107-p502/P5-02/training-reviewed-v2-gpu/metrics.json`.

### Active three-class P5-02 qualification (2026-11-12)

The trained EMA export from `all_outputs.zip` passed the frozen three-class
gates. On validation (746 rows), mIoU was 0.5926, with IoU 0.9469 for
regolith, 0.3148 for rock, and 0.5162 for shadow. The one-time held-out test
(749 rows) passed at mIoU 0.6057, with IoU 0.9181, 0.3691, and 0.5300
respectively. The checkpoint SHA-256 is
`16276d1375f023d04d3f60557a3c6a84f3d5dbfc97c51d4aea3deeec8c5a8210`.

The evaluation preserved the existing scene splits and the gate thresholds
(mIoU >= 0.50; each class IoU >= 0.25). Crater pixels were ignored in this
three-class model, and crater safety remains the geometry layer's responsibility.
The full confusion matrices, class support, gate calculations, frozen input
hashes, and evaluator are in
`evidence/runs/20261112-p502-threeclass/P5-02/`. This result completes P5-02
for the active analog engineering scope; it does not establish deployment
performance. The historical five-class run above remains failed.

### New downloaded training sources (2026-11-10)

The user supplied Roboflow COCO/YOLO polygon exports, LunarSeg, and AI4Mars
merged 0.6 archives. They are staged on D: with archive SHA-256 provenance.
The new canonical corpus and per-run source/split policies live outside Git
under `D:\data\lunabot-datasets\canonical-20261110`. New source rows are
training-only; the prior validation/test scenes and numeric gates are retained.
The corpus audit is separate from model qualification. No new trained score or
qualification pass is claimed by preparing these files.

See [D: training workflow](d-drive-training.md) for mappings, excluded subsets,
the Kaggle notebook, upload bundle, architecture, and remaining evaluation
limitations. The notebook starts a fresh pretrained RGB ResNet34 U-Net and
leaves test scoring disabled during development.
