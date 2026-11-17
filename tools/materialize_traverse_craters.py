"""Materialize exact-frame partial annotations for a separate training scene.

Polygons use the 500x500 review coordinate system. Unreviewed pixels remain
ignore. No labels are propagated to a different exposure or camera pose.
The published height map distinguishes the depressions from intervening mounds.
"""
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

ROOT=Path(__file__).resolve().parents[1]
# Conservative illuminated inner-wall patches and separate cast-shadow patches.
REVIEWS={
 0:{3:[[192,252],[232,245],[269,249],[272,254],[224,262],[186,266]],
    4:[[238,269],[268,265],[263,270],[242,273]],
    0:[[332,294],[353,292],[354,304],[333,306]]},
 2:{3:[[155,315],[199,303],[247,297],[286,306],[264,316],[210,327],[155,341]],
    4:[[227,355],[293,342],[315,356],[288,380],[236,382]],
    0:[[370,310],[388,315],[386,330],[370,326]]},
 4:{3:[[201,282],[225,276],[253,275],[265,279],[243,284],[210,290]],
    4:[[236,292],[270,288],[280,292],[270,300],[245,301]],
    0:[[362,321],[379,325],[376,340],[360,336]]},
 6:{3:[[156,421],[184,408],[218,402],[252,406],[232,414],[200,419],[160,435]],
    4:[[214,450],[256,438],[276,449],[263,473],[224,475]],
    0:[[302,352],[320,354],[320,366],[303,367]]},
 8:{3:[[211,239],[242,237],[266,241],[273,245],[241,246],[217,246]],
    4:[[231,252],[264,251],[272,256],[253,264],[231,263]],
    0:[[310,330],[330,331],[330,341],[311,342]]},
 10:{3:[[212,319],[231,312],[256,314],[273,320],[256,324],[232,326],[213,332]],
     4:[[234,345],[265,339],[280,346],[269,360],[242,360]],
     0:[[335,371],[355,371],[354,382],[336,381]]},
}


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    acquired=ROOT/'.build/p502-traverse'
    output=ROOT/'.build/p502-traverse-annotated-v1'
    output.mkdir(exist_ok=False)
    members={r['member']:r for r in json.loads((acquired/'acquired.json').read_text())['members']}
    rows,provenance=[],[]
    contact=Image.new('RGB',(1500,1000))
    for i,(position,polygons) in enumerate(REVIEWS.items()):
        name=f'View1/Traverse1/{position:02d}m/loc{position*2}_camL_025ms.png'
        record=members[name];path=Path(record['path'])
        if sha(path)!=record['sha256']:raise ValueError('source hash changed')
        with Image.open(path) as image:
            rgb=image.convert('RGB');gray=np.asarray(image.convert('L'));size=image.size
        target=Image.new('L',size,255);draw=ImageDraw.Draw(target)
        for class_id,polygon in polygons.items():
            draw.polygon([(round(x*size[0]/500),round(y*size[1]/500)) for x,y in polygon],fill=class_id)
        mask=np.asarray(target).copy()
        # Saturated/underexposed wall and soil pixels carry no trustworthy texture.
        mask[((mask==3)|(mask==0)) & ((gray<25)|(gray>240))]=255
        mask[(mask==4)&(gray>60)]=255
        destination=output/f'position-{position:02d}-mask.png'
        Image.fromarray(mask).save(destination)
        rows.append(dict(image=str(path),mask=str(destination),source_id='nasa-polar-traverse-crater-train',terrain_id='sservi-traverse-2023',split='train'))
        overlay=np.asarray(rgb).copy()
        colors={0:[0,255,255],3:[255,0,255],4:[0,120,255]}
        for class_id,color in colors.items():
            selected=mask==class_id
            overlay[selected]=(overlay[selected]*.35+np.array(color)*.65).astype(np.uint8)
        reviewed=Image.fromarray(overlay)
        reviewed.save(output/f'position-{position:02d}-overlay.png')
        contact.paste(reviewed.resize((500,500)),((i%3)*500,(i//3)*500))
        provenance.append(dict(member=name,image_sha256=sha(path),mask_sha256=sha(destination),
            polygons_review_500px=polygons,pixel_support={str(c):int((mask==c).sum()) for c in [0,3,4]},
            reviewer='codex-on-behalf-of-jay',scope='exact 25ms frame; partial inner wall, cast shadow, and soil only'))
    with (output/'manifest.csv').open('w',newline='') as stream:
        w=csv.DictWriter(stream,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    contact.save(output/'review-contact.png')
    (output/'review.json').write_text(json.dumps(dict(source='https://ti.arc.nasa.gov/dataset/PolarTrav/',
        geometric_review='Published processed FARO height map and camera-location diagram, arXiv:2403.12194 Fig.3.',
        entire_scene_training_only=True,propagation_to_other_exposures=False,reviews=provenance,
        limitations=['Six related views of a single constructed scene.','Sparse partial masks; no deployment qualification claim.']),indent=2)+'\n')
    print(json.dumps(provenance,indent=2))


if __name__=='__main__':main()
