"""Create a new, traceable corpus; never mutate acquired or previous run files."""
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image

from train_perception import audit_class_support, sha256_file

ROOT=Path(__file__).resolve().parents[1]


def main():
    prior=ROOT/'.build/p502-cache-256'
    output=ROOT/'.build/p502-cache-reviewed-v1'
    native=ROOT/'.build/p502-corpus-repair-v1'
    output.mkdir(exist_ok=False);native.mkdir(exist_ok=False)
    old=json.loads((prior/'cache-report.json').read_text())
    lineage={hashlib.sha256(r['source_image'].encode()).hexdigest():r for r in old['lineage']}
    rows,origins,changes,excluded=[],[],[],[]
    converted={}
    def add(row,image,mask,cached_image=None,cached_mask=None,repeat=1):
        image,mask=Path(image),Path(mask)
        key=hashlib.sha256(str(image).encode()).hexdigest()
        if key not in converted:
            imout,maout=output/(key+'.png'),output/(key+'-mask.png')
            if cached_image:
                imout.symlink_to(cached_image)
            else:
                with Image.open(image) as im:im.convert('RGB').resize((256,256),Image.Resampling.BILINEAR).save(imout)
            if cached_mask:
                maout.symlink_to(cached_mask)
            else:
                with Image.open(mask) as ma:ma.resize((256,256),Image.Resampling.NEAREST).save(maout)
            # Keep lexical cache path: resolve() would dereference the symlink
            # and lose the key that links a cached pair to its source record.
            converted[key]=(str(imout),str(maout))
            origins.append(dict(source_image=str(image),source_mask=str(mask),
                source_image_sha256=sha256_file(image),source_mask_sha256=sha256_file(mask),
                image_sha256=sha256_file(imout),mask_sha256=sha256_file(maout)))
        imout,maout=converted[key]
        rows.extend([{**row,'image':imout,'mask':maout} for _ in range(repeat)])
    for row in csv.DictReader((prior/'manifest.csv').open()):
        cached_image,cached_mask=Path(row['image']),Path(row['mask'])
        origin=lineage[cached_image.stem]
        with Image.open(cached_image) as im:
            a=np.asarray(im)
        if int(a.max())==int(a.min()):
            excluded.append(dict(split=row['split'],source_image=origin['source_image'],value=int(a.min())))
            continue
        mask=Path(origin['source_mask'])
        if row['split']=='train' and row['terrain_id']=='terrain05':
            with Image.open(mask) as ma:values=np.asarray(ma).copy()
            count=int((values==3).sum())
            if count:
                values[values==3]=255
                repaired=native/(cached_image.stem+'-quarantined-mask.png')
                Image.fromarray(values).save(repaired)
                changes.append(dict(image=origin['source_image'],old_mask=str(mask),old_mask_sha256=sha256_file(mask),
                    new_mask=str(repaired),new_mask_sha256=sha256_file(repaired),crater_pixels_ignored=count))
                mask=repaired;cached_mask=None
        add(row,origin['source_image'],mask,cached_image,cached_mask)
    annotated=ROOT/'.build/p502-traverse-annotated-v1'
    crop_lineage=[]
    for row in csv.DictReader((annotated/'manifest.csv').open()):
        add(row,row['image'],row['mask'])
        with Image.open(row['image']) as im,Image.open(row['mask']) as ma:
            rgb=im.convert('RGB');target=ma.copy();array=np.asarray(target)
        yy,xx=np.where(array==3)
        if not len(xx):continue
        cx,cy=float(xx.mean()),float(yy.mean())
        for side in [512,768,1024]:
            for offset in [-.15,0,.15]:
                left=max(0,min(rgb.width-side,round(cx-side/2+side*offset)))
                top=max(0,min(rgb.height-side,round(cy-side/2)))
                box=(left,top,left+side,top+side)
                name=Path(row['mask']).stem+f'-crop-{side}-{offset}'
                image_path,mask_path=native/(name+'.png'),native/(name+'-mask.png')
                rgb.crop(box).save(image_path);target.crop(box).save(mask_path)
                add(row,image_path,mask_path,repeat=8)
                crop_lineage.append(dict(image=str(image_path),mask=str(mask_path),
                    parent_image=row['image'],parent_mask=row['mask'],crop_box=list(box),
                    scene=row['terrain_id'],split='train',training_repetitions=8))
    with (output/'manifest.csv').open('w',newline='') as stream:
        w=csv.DictWriter(stream,fieldnames=['image','mask','source_id','terrain_id','split']);w.writeheader();w.writerows(rows)
    splits={s:[r for r in rows if r['split']==s] for s in ['train','validation','test']}
    support=audit_class_support(splits)
    document=dict(input_manifest_sha256=sha256_file(prior/'manifest.csv'),image_size=256,
        records={k:len(v) for k,v in splits.items()},resized_class_pixel_support=support,lineage=origins)
    (output/'cache-report.json').write_text(json.dumps(document,indent=2)+'\n')
    report=dict(excluded_constant_frames=excluded,quarantined_train_masks=changes,training_crops=crop_lineage,
        validation_and_test_nonconstant_labels_unchanged=True,originals_mutated=False,
        records=document['records'],pixel_support=support,manifest_sha256=sha256_file(output/'manifest.csv'))
    (native/'repair-report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ['records','pixel_support','manifest_sha256']},indent=2),flush=True)


if __name__=='__main__':main()
