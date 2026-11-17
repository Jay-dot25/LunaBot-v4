# Train the P5-02 three-class model in Kaggle

The active P5-02 target classes are regolith, rock, and shadow. The upload ZIP
is built from the complete 33,599-row corpus; bedrock is merged into rock,
crater pixels become ignore, and source train/validation/test assignments are
preserved. The separate five-class archive remains unchanged.

1. Upload `.build/p502-three-class-kaggle.zip` as a **private Kaggle Dataset**.
   Wait for Kaggle to finish processing it.
2. Create a private Kaggle Notebook from
   `notebooks/p502-three-class-kaggle.ipynb`, attach that dataset, and enable a
   GPU.
3. Run the first session with `RESUME=False`, `TARGET_EPOCH=1`, and
   `RUN_TEST=False`. Save the notebook version after it finishes and retain
   `p502-three-class-run-output.zip`.
4. For later sessions, attach the previous notebook output (or its run ZIP),
   set `RESUME=True`, and raise `TARGET_EPOCH` to the desired absolute epoch.
   Keep the same dataset version, batch size, image size, workers, and source
   code. Continue to epoch 30.
5. In the final session only, set `TARGET_EPOCH=30` and `RUN_TEST=True`. The
   trainer evaluates the held-out test split once and writes `metrics.json`.
   The frozen gates are mIoU >= 0.50 and per-class IoU >= 0.25. A failed gate
   means the model is not qualified for integration.

The notebook starts with `BATCH_SIZE=8`, `IMAGE_SIZE=256`, and `WORKERS=2`.
If Kaggle reports GPU memory pressure, resume the latest saved checkpoint with
batch size 4. If workers fail, use 0. Do not change the frozen dataset or
qualification thresholds between sessions.
