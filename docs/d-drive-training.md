# D: project and downloaded-data training (2026-11-10)

Active ROS workspace: `/home/jay/lunabot-v4`. Ubuntu's approved storage
destination is `D:\WSL\Ubuntu-22.04`, keeping Linux paths and environments
intact. `D:\lunabot-v4` (`/mnt/d/lunabot-v4`) is the Windows-accessible
project snapshot used to prepare the Kaggle artifacts.
Datasets: `D:\data\lunabot-datasets`. Original archives, extracted sources,
canonical masks, and training outputs stay outside Git. See `staging.json`
for archive SHA-256 values and exact extraction locations.

The finalized corpus has 32,104 train / 746 validation / 749 test records.
New training rows: 7,704 polygon images, 3,084 LunarSeg images and 15,899
AI4Mars NavCam images. The `audit.json` result is the required integrity check
before packaging; it is not a trained-model qualification result.

The downloaded Roboflow COCO v1 and YOLO v2 archives are overlapping exports,
not independent datasets. The converter uses training polygons from both,
removes exact RGB duplicates at a common 256px resolution, and leaves
unannotated pixels as 255. This check does not establish scene independence
or remove all near duplicates. Source test/validation exports are not used.

LunarSeg comes from https://github.com/nubot-nudt/LuSeg. The upstream trainer
orders its three labels as background, negative obstacle, positive obstacle.
The conversion maps 1 to crater (negative obstacle), 2 to rock, and background
to ignore. This obstacle taxonomy is an approximation to the project's
impact-crater taxonomy and needs representative semantic review; it must not
be presented as new crater qualification ground truth. Only its published
training list is used; depth/color labels and other split files remain staged.

AI4Mars NAV labels map soil/sand to regolith, bedrock to bedrock, big rock to
rock, and NULL to ignore. The converter considers explicit training
labels from MSL NavCam only. MastCam contains an undefined label 4 and is
excluded until its ontology is established. MER's explicitly unmasked labels
remain staged until rover/range masking is reviewed. M2020 GEO classes are
a different ontology and are excluded.
Expert test masks are excluded. Image pairing and unmapped IDs are checked.
Retain the archive's `info.md` and `label_keys.json` with any derived dataset.

All new sources are training-only. Existing reviewed validation/test masks
and scenes are retained. Therefore the old sparse held-out class support
remains a limitation. No score gate or qualification claim changes here.

## Train in Kaggle

1. In Kaggle Datasets, create a private dataset and upload
   `D:\data\lunabot-kaggle\kaggle-p502-new-datasets.zip` (3.83 GB). Wait for
   processing to finish. The notebook can use either the mounted ZIP or
   Kaggle's unpacked files. The local archive SHA-256 is
   `a41c2b40b2b4fe0e83abe81645c0e334d838ce812f378c5e8c52c46746d3c730`.
2. Create a private Kaggle Notebook and import the current workspace copy of
   `notebooks/p502-new-datasets-kaggle.ipynb` from the File menu. Under
   **Add Input**, attach the private dataset. In **Settings**, select a GPU.
   A single GPU is used. The verified ImageNet encoder weights are included,
   so Internet can be off.
3. In the first code cell, keep `RESUME=False`, `STOP_AFTER_EPOCH=1`, and
   `RUN_TEST=False`. Use **Save Version → Save & Run All** with GPU and wait
   for that version to finish. Check the logs for `PREFLIGHT PASS` and
   `Completed epoch 1/40`. The notebook reports elapsed epoch time and
   estimated time remaining. The first epoch is part of the actual 40-epoch run.
4. Confirm the completed version's output contains
   `p502-new-datasets-run-01/last.pt`, `best.pt`, and `history.json`. For the
   next chunk, open a copy of the notebook, keep the same training dataset
   attached, and attach that completed notebook version's output through
   **Add Input**. Set `RESUME=True` and raise `STOP_AFTER_EPOCH` to an
   absolute epoch number. For example, at 40 minutes per epoch, setting it to
   10 after epoch 1 schedules about 6 hours of training. Leave time for setup,
   validation, and output saving within your available GPU session and quota.
   Save & Run All again. Repeat with the latest successful output until
   epoch 40. If the notebook finds multiple prior runs, set
   `RESTORE_RUN_FROM` to the intended checkpoint directory printed in the error.
5. Keep the dataset version, GPU setup, notebook code, batch size, image size,
   and worker count fixed across chunks. If the first run runs out of GPU
   memory, start a fresh version with `BATCH_SIZE=4`. If the data loader reports
   a worker or shared-memory failure, start a fresh version with `WORKERS=0`.
   Keep the highest completed saved version if a later session expires.
6. Inspect `validation-metrics.json` for the selected epoch, mIoU, and
   class IoUs. `best.pt` is the validation-selected inference checkpoint.
   `last.pt` contains optimizer and training state for resuming. Set
   `EXPORT_ZIP=True` in the final chunk to also save a delivery ZIP with the
   checkpoints, metrics, source code, configuration, manifest, and environment.
   Keep the original private dataset available with that delivery.
7. For the final engineering test, set `RUN_TEST=True` in the chunk ending at
   epoch 40. Test scoring proceeds only if the selected checkpoint passes the
   validation gates. Read `test-metrics.json` for the result. The inherited
   test scenes have previously been inspected and cannot establish independent
   model qualification.

Kaggle documents dataset uploads, input attachments, GPU setup, and saved
notebook outputs in its [dataset guide](https://www.kaggle.com/docs/datasets)
and [notebook guide](https://www.kaggle.com/docs/notebooks).

Build and inspect the corpus in WSL:

```bash
cd /mnt/d/lunabot-v4
.build/p502-training-venv/bin/python tools/materialize_downloaded_training.py \
  --output /mnt/d/data/lunabot-datasets/canonical-20261110
```

The corpus directory contains the manifest, extended source/split policies,
canonical pairs, and `materialization.json` with lineage and rejection counts.
Do not edit a corpus after starting training.

Fresh development training:

```bash
export TMPDIR=/mnt/d/data/lunabot-tmp
export TORCH_HOME=/mnt/d/data/lunabot-torch-cache
export PIP_CACHE_DIR=/mnt/d/data/lunabot-pip-cache
export PYTHONPYCACHEPREFIX=/mnt/d/data/lunabot-pycache
.build/p502-training-venv/bin/python -u tools/train_downloaded_resnet.py \
  --corpus /mnt/d/data/lunabot-datasets/canonical-20261110 \
  --gates evidence/runs/20261107-p502/P5-02/gates-frozen.json \
  --output /mnt/d/data/lunabot-training/p502-resnet34-20261110 \
  --epochs 40 --batch-size 2 --size 320
```

Resume with the identical command plus `--resume`. The output must retain
both `best.pt` and `last.pt`. Dataset/configuration/code hashes must match.
The trainer saves checkpoints after each completed epoch and logs batch
progress; an interrupted first epoch has no resumable checkpoint yet.

This is a fresh RGB ResNet34 U-Net, not the small U-Net in
`train_perception.py`. Input is RGB /255 with ImageNet mean/std. No rare-class
copy-paste, TTA, or validation-fitted calibration is applied. It uses AMP,
source-balanced sampling, weighted cross-entropy plus masked Dice, encoder/decoder
learning rates with warmup/cosine decay, and EMA; validation selects
the best five-class mIoU. Sparse validation targets are evaluated at their
stored mask resolution. The test split is preflighted for class support,
but no test predictions are computed. Preserve the exact trainer with the
checkpoint; the small-U-Net evaluator cannot load this architecture.

Qualification still requires a frozen final inference pipeline, independent
held-out evidence, mIoU >=0.50 and every class IoU >=0.25. Validation-only
development is not qualification, even when a validation threshold is met.
