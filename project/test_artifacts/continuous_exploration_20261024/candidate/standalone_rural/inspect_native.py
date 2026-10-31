"""Read only small native format examples; never extract stock art or keys."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile


def inspect(home):
    evidence = []
    for level in ('smallgrid', 'template', 'italy'):
        with zipfile.ZipFile(home / 'content' / 'levels' / (level + '.zip')) as z:
            candidates = [i for i in z.infolist() if i.file_size < 45000 and
                          (i.filename.endswith(('info.json', 'main.level.json', 'items.level.json', 'main.materials.json')))]
            selected = []
            for i in candidates:
                if (level != 'italy' and '/art/' not in i.filename) or any(t in i.filename for t in ('spawnpoints/', 'main.level.json', '/info.json', 'environment/')):
                    selected.append(i)
            for i in selected[:24]:
                text = z.read(i).decode('utf-8-sig')
                evidence.append(dict(archive=level + '.zip', member=i.filename,
                                     sha256=hashlib.sha256(z.read(i)).hexdigest(), text=text))
                print(i.filename + '\n' + text[:18000].encode('ascii', 'backslashreplace').decode('ascii'))
    out = Path(__file__).resolve().parent / 'evidence'
    out.mkdir(exist_ok=True)
    (out / 'native_formats.json').write_text(json.dumps(evidence, indent=2), encoding='utf-8')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--home', type=Path, required=True)
    inspect(p.parse_args().home)
