"""Freeze reviewed evaluation masks before any model scores are inspected.

Unreviewed regolith-class pixels in known crater scenes are ignored rather
than counted as negative crater examples. Other scenes retain their masks.
"""
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from train_perception import audit_class_support,sha256_file

ROOT=Path(__file__).resolve().parents[1]
POLYGONS={
 'terrain11':[[1290,654],[1350,620],[1415,596],[1465,602],[1460,625],[1400,658],[1340,689],[1290,701]],
 'terrain12':[[1090,590],[1200,565],[1270,530],[1300,535],[1265,570],[1170,620],[1100,620]],
}


def main():
    prior=ROOT/'.build/p502-cache-reviewed-v1'
    out=ROOT/'.build/p502-cache-reviewed-v2'
    if (out/'cache-report.json').exists():raise ValueError('completed output already exists')
    out.mkdir(exist_ok=True)
    native=ROOT/'.build/p502-evaluation-review-v2';native.mkdir(exist_ok=True)
    report=json.loads((prior/'cache-report.json').read_text())
    origins={hashlib.sha256(r['source_image'].encode()).hexdigest():r for r in report['lineage']}
    rows=list(csv.DictReader((prior/'manifest.csv').open()))
    staged=Path('/home/jay/lunabot-p502-persistent')
    source=staged/'materialized-20261105-002022'
    original={Path(r['image']).name:r for r in csv.DictReader((source/'manifest.csv').open())}
    details=[];new_origins={}
    for row in rows:
        key=Path(row['image']).stem;origin=origins[key].copy()
        imout,maout=out/(key+'.png'),out/(key+'-mask.png')
        if not imout.exists():imout.symlink_to(row['image'])
        if row['split']!='train' and row['terrain_id'] in POLYGONS:
            terrain=row['terrain_id'];filename=Path(origin['source_image']).name
            exact=filename==f'{int(terrain[-2:])}_A_off_180_L_0032.png'
            with Image.open(origin['source_mask']) as im:values=np.asarray(im).copy()
            ignored=int(((values==0)|(values==3)).sum())
            values[(values==0)|(values==3)]=255
            accepted=0
            if exact:
                original_mask=source/original[filename]['mask']
                candidate=staged/'crater-candidates-delegated-20261106/candidate_masks'/terrain/filename
                with Image.open(original_mask) as im:original_values=np.asarray(im)
                with Image.open(candidate) as im:projected=np.asarray(im)
                with Image.open(origin['source_image']) as im:gray=np.asarray(im.convert('L'))
                polygon=Image.new('L',(values.shape[1],values.shape[0]))
                ImageDraw.Draw(polygon).polygon([tuple(p) for p in POLYGONS[terrain]],fill=1)
                selected=(np.asarray(polygon)==1)&(original_values==0)&(projected==3)&(gray>=40)&(gray<=235)
                values[selected]=3;accepted=int(selected.sum())
                with Image.open(origin['source_image']) as im:overlay=np.asarray(im.convert('RGB')).copy()
                overlay[selected]=(overlay[selected]*.35+np.array([255,0,255])*.65).astype(np.uint8)
                Image.fromarray(overlay).save(native/(terrain+'-review-overlay.png'))
                if accepted<100:raise ValueError('reviewed polygon lacks geometry-supported pixels')
            mask=native/(key+'-mask.png');Image.fromarray(values).save(mask)
            Image.fromarray(values).resize((256,256),Image.Resampling.NEAREST).save(maout)
            origin.update(source_mask=str(mask),source_mask_sha256=sha256_file(mask),mask_sha256=sha256_file(maout))
            details.append(dict(image=origin['source_image'],split=row['split'],ignored_ambiguous_pixels=ignored,
                accepted_crater_pixels=accepted,reviewed_polygon=POLYGONS[terrain] if exact else None,
                old_mask_sha256=sha256_file(Path(origins[key]['source_mask'])),new_mask_sha256=sha256_file(mask)))
        elif not maout.exists():maout.symlink_to(row['mask'])
        new_origins[key]=origin
        row.update(image=str(imout),mask=str(maout))
    with (out/'manifest.csv').open('w',newline='') as stream:
        w=csv.DictWriter(stream,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    splits={s:[r for r in rows if r['split']==s] for s in ['train','validation','test']}
    support=audit_class_support(splits)
    report.update(input_manifest_sha256=sha256_file(prior/'manifest.csv'),lineage=list(new_origins.values()),
                  resized_class_pixel_support=support)
    (out/'cache-report.json').write_text(json.dumps(report,indent=2)+'\n')
    (native/'review.json').write_text(json.dumps(dict(frozen_before_training_and_scores=True,
        reviewer='codex-on-behalf-of-jay',reviews=details,pixel_support=support,
        limitation='Crater evaluation remains two sparse exact-frame patches; unrelated pixel support is not evidence of scene-level crater generalization.'),indent=2)+'\n')
    print(json.dumps(dict(pixel_support=support,manifest_sha256=sha256_file(out/'manifest.csv')),indent=2))


if __name__=='__main__':main()
