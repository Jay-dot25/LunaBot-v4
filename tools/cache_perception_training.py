"""Cache resized RGB/indexed-mask pairs with lineage for a frozen input CSV."""
import argparse
import csv
import hashlib
import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
from PIL import Image

from train_perception import ROOT, load_records, sha256_file, audit_class_support


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--image-size', type=int, default=256)
    args = parser.parse_args()
    records = load_records(args.manifest, args.manifest.parent,
                           ROOT/'config/perception/splits.json', ROOT/'config/perception/sources.json')
    args.output.mkdir(parents=True, exist_ok=False)
    def convert(row):
        key = hashlib.sha256(row['image'].encode()).hexdigest()
        image_path, mask_path = Path(row['image']), Path(row['mask'])
        image_out, mask_out = args.output/(key+'.png'), args.output/(key+'-mask.png')
        with Image.open(image_path) as image, Image.open(mask_path) as mask:
            if image.size != mask.size or mask.mode not in ('L','P'):
                raise ValueError(f'image/mask mismatch: {image_path}')
            values = set(np.unique(np.asarray(mask)).tolist())
            if values - {0,1,2,3,4,255}:
                raise ValueError(f'unsupported mask IDs: {mask_path}')
            image.convert('RGB').resize((args.image_size,)*2,Image.Resampling.BILINEAR).save(image_out)
            mask.resize((args.image_size,)*2,Image.Resampling.NEAREST).save(mask_out)
        lineage = {'source_image': str(image_path), 'source_mask': str(mask_path),
                   'source_image_sha256': sha256_file(image_path), 'source_mask_sha256': sha256_file(mask_path),
                   'image_sha256': sha256_file(image_out), 'mask_sha256': sha256_file(mask_out)}
        return {**row,'image':str(image_out.resolve()),'mask':str(mask_out.resolve())},lineage
    rows = [row for values in records.values() for row in values]
    with ThreadPoolExecutor(max_workers=4) as executor:
        converted = list(executor.map(convert,rows))
    with (args.output/'manifest.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=['image','mask','source_id','terrain_id','split'],extrasaction='ignore')
        writer.writeheader();writer.writerows(row for row,_ in converted)
    resized={split:[row for row,_ in converted if row['split']==split] for split in records}
    support=audit_class_support(resized)
    report={'input_manifest_sha256':sha256_file(args.manifest),'image_size':args.image_size,
            'records':{s:len(r) for s,r in resized.items()},'resized_class_pixel_support':support,
            'lineage':[lineage for _,lineage in converted]}
    (args.output/'cache-report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='lineage'},indent=2))


if __name__=='__main__':
    main()
