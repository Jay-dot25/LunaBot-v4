# P5-02 dataset remediation

## November 12 corpus revision

Build the mask revision from the extracted local bundle:

```bash
python tools/revise_perception_corpus.py \
  .build/p502-local-20261111/corpus .build/p502-corpus-20261112-v1
```

The revised manifest references the original images and unmodified masks. Keep
both directories available. On POLAR-Sim training and validation frames where
at least half the regolith-labeled area is near white (all RGB channels at
least 250), change those near-white regolith pixels to ignore (255). Existing
positive labels are preserved. The revision ledger records each changed mask.
The test split retains its original masks and assignments.

“Image 350” can refer to row 350 or index 350 of either evaluation split.
Validation row 350 (one-based), Terrain09 `09_A_off_270_R_2048.png`, is
heavily overexposed. The other three candidates already ignore ambiguous
terrain. This revision handles clipping. Visible crater-like depressions that
are not clipped still need boundary review.

### Lunar annotation queue

The local pilot at `.build/p502-lunar-annotation-pilot/pilot.json` records
11 source frames, their SHA-256 hashes, provenance, scene assignments, and
six draft masks at source resolution. Orange in `draft-overlay-contact.png`
marks crater and cyan marks bedrock. The polygons mark selected interior
patches, with all other pixels ignored. The drafts were traced manually.

- Shorty and Van Serg have four draft crater masks across two ground-level
  sites. Keep the three Shorty frames in one scene. [Shorty panorama frame
  index](https://www.lpi.usra.edu/resources/apollopanoramas/pans/?pan=JSC2007e045388)
  and [Apollo 17 frame catalog](https://apollojournals.org/alsj/a17/images17.html).
- Bessel and Lichtenberg have one draft bedrock mask each. These orbital
  close-ups span about 500 m and 530 m respectively, so use them as candidates
  for auxiliary training after review. [Bessel outcrops](https://science.nasa.gov/photojournal/lava-flows-exposed-in-bessel-crater/)
  and [Lichtenberg crater wall](https://science.nasa.gov/photojournal/lichtenberg-crater/).
- The two East Massif traverse frames show the apparent exposure at a
  distance where its pixel boundary is unresolved. Their ledger entries
  record that exclusion. [Apollo 17 frame catalog](https://apollojournals.org/alsj/a17/images17.html).

The pilot is pending boundary review and has no split assignments. The revised
corpus has no dependency on the draft images or masks.

| Candidate scene | Class to review | Acquisition reference |
| --- | --- | --- |
| Apollo 15 Hadley Rille | Bedrock | [NASA terrain photo index](https://www.nasa.gov/wp-content/uploads/static/history/alsj/WOTM/WOTM-Terrain.html), Station 9A outcrop sequences AS15-89-12015–12027 and AS15-89-12045–12056. Group overlapping outcrop views with the existing Hadley Rille validation scene. |
| Apollo 17 Shorty Crater | Crater | [Station 4 mission report](https://www.nasa.gov/wp-content/uploads/static/history/alsj/a17/A17_MissionReport.pdf), south rim views. |
| Apollo 17 Camelot Crater | Crater | [NASA EVA account](https://www.nasa.gov/history/50-years-ago-apollo-17-lands-at-taurus-littrow/), Station 5 southwest rim. |
| Apollo 17 Van Serg Crater | Crater | [NASA photographic summary, figure 4-35](https://www.nasa.gov/wp-content/uploads/static/history/alsj/a17/as17psr.pdf), AS17-142-21801, 21805, 21807 and 21811. |

Keep source frame IDs, site IDs and links alongside annotations. Use SAM to
propose boundaries and verify them against the source imagery and geology.
Trace visible crater interiors and exposed continuous outcrop. Use ignore for
occluded or uncertain boundaries. A panorama and its component frames count as
one scene. Detached boulders and talus belong to rock. Apollo telephoto and
orbital views need a separate scale check against the rover camera.

### Scene evaluation and experiments

Use the revision report's positive-frame counts to identify missing coverage.
Group every view of the same physical site, including cross-source derivatives,
before assigning scenes. Reserve new independent test sites, then run
leave-scene-out folds over the remaining sites. Train a separate model per fold
and predict only its held-out scene. Keep Mars and synthetic training data out
of the held-out lunar metrics. Use native mask coordinates for scoring.

The final evaluator now reports scene confusion matrices and percentile
intervals from resampling whole scenes. It withholds intervals with fewer than
three positive scenes or fewer than 95% of draws containing class support.
With a small scene count, interpret any resulting interval as exploratory.

For domain adaptation, compare the baseline against train-only histogram
matching after acquiring lunar training scenes. Derive the reference from
training scenes within each fold. Use native-resolution source imagery for a
448-pixel or larger crop experiment. Upsampling the cached 256-pixel frames
cannot recover crater detail. Compare Tversky against the baseline with the
same folds, then fit any class bias only on separate validation scenes with
adequate support.

## Decision

Do not append newly found public datasets to the qualification corpus yet. None
is a drop-in source for the missing independent, pixel-labeled crater and
bedrock evaluation data. The current corpus and frozen split remain unchanged.

## Current gap

The audited 2026-11-10 corpus has 746 validation and 749 test records. All five
classes have nonzero support, but crater and bedrock evaluation is too sparse
and scene-limited to support strong class claims:

| Class | Validation pixels | Test pixels | Limitation |
| --- | ---: | ---: | --- |
| Bedrock | 83 | 11,149 | Validation is two related Apollo frames from one Hadley Rille scene; test is 48 BASEPROD frames from one Bardenas site. |
| Crater | 249 | 215 | Each split has reviewed pixels in one exact frame; the ambiguous remainder is ignored. |
| Shadow | 453,578 | 615,255 | The 2026-11-10 added training sources contribute zero shadow pixels. |

Counts are from `.build/p502-local-bundle/config/audit.json`, frozen support
limitations from `.build/p502-local-bundle/config/gates-frozen.json`, and new
source support from `.build/p502-local-bundle/config/materialization.json`.

## Public-source triage

- **Chang'e 5 Landing Camera Crater Detection Dataset** (Zenodo
  `10.5281/zenodo.11578934`): 132 hand-labeled descent images, with crater rims
  represented by ellipses. The record requests citation but does not state a
  license. Ellipses are not reviewed pixel masks, image scale changes during
  descent, and the sampled testing images are from the same landing sequence.
  Do not add to five-class qualification without rights clarification and
  separate scene/scale suitability review.
- **MoonAnything** (Zenodo `10.5281/zenodo.18415119`): real LRO imagery is
  paired with depth and geometry, but the listed release does not provide the
  needed regolith/bedrock/rock/crater/shadow semantic masks; the Zenodo rights
  field is blank. Its multi-gigabyte archives are not justified for this gap.
- **ReSyRIS** (DLR; Zenodo `10.5281/zenodo.7691201`): useful real lunar-analog
  rock-instance masks, but its real test uses repeated configurations of the
  same 36 rocks at one Etna site. It does not provide crater or bedrock masks,
  so it cannot repair the principal held-out gaps.
- **Apollo Surface Panoramas** (NASA/LPI): relevant lunar imagery, but the
  supplied annotations identify major geographic features rather than dense
  pixel classes. New pixel masks would need careful manual annotation and
  scene-level assignment. LPI says the site is generally CC BY 4.0 unless
  otherwise noted, but that does not make the existing feature annotations
  segmentation masks. A specific Apollo 16 Station 11 panorama shows white
  breccia boulders; the Apollo 16 preliminary geology report says no definite
  outcrops or ledges were observed in the traverse area. That view may support
  rock review, but it is not an evidence-backed bedrock source. Do not download
  it for the current bedrock gap.

## Required remediation

1. Obtain or collect multiple independent lunar/analog scenes that visibly
   contain crater and continuous exposed bedrock. Record source rights,
   site/scene IDs, image scale, and acquisition provenance before ingestion.
2. Have a qualified reviewer annotate full-resolution crater and bedrock
   masks in scenes reserved for validation and test. Keep uncertain/occluded
   pixels ignored. Do not propagate annotations to other exposures unless each
   variant is reviewed.
3. Add representative training labels for shadow from an approved source if
   shadow performance is in scope; the new 2026-11-10 sources add none.
4. Audit scene grouping and near duplicates across all source exports before
   assigning any new examples. Keep the existing frozen test images untouched.
5. Materialize a new versioned corpus and re-run integrity, class-support,
   provenance, and split audits. Only then run the complete planned training
   budget and score the held-out test once.

The existing small crater/bedrock support is not a substitute for this work,
and the current one-epoch ResNet34 run is development-only. Do not use it as
qualification evidence.

## Source references

- Chang'e 5 labels: https://zenodo.org/records/11578934
- MoonAnything: https://zenodo.org/records/18415119
- ReSyRIS: https://www.dlr.de/en/rm/research/publications-and-downloads/datasets/resyris
- Apollo Surface Panoramas: https://catalog.data.gov/dataset/apollo-surface-panoramas
- LPI Apollo panorama archive and terms: https://www.lpi.usra.edu/resources/apollopanoramas/
- Apollo 16 preliminary geology report: https://www.lpi.usra.edu/resources/USGS-Reports/Astro-0048.pdf
