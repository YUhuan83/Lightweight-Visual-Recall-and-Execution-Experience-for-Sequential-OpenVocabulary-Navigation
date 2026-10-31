"""Verify every original archive member and extract readable evidence without overwrites."""
import hashlib
import json
from pathlib import Path
import tarfile

BASE=Path(__file__).resolve().parent
ARCHIVE=BASE/'town_memory_validation_20261014_evidence.tar.gz'
EXPECTED='168fa598892c2c18e48f9d5a4e04b6a2d41d9a1d6ea5758b4fa8dda9934d4659'


def main():
    digest=hashlib.sha256(ARCHIVE.read_bytes()).hexdigest()
    if digest!=EXPECTED: raise RuntimeError('Transport archive hash mismatch')
    out=BASE/'evidence'
    with tarfile.open(ARCHIVE,'r:gz') as archive:
        manifest=json.load(archive.extractfile('evidence_manifest.json'))
        expected={x['path']:x for x in manifest['files']}
        members=archive.getmembers()
        if {x.name for x in members}!=set(expected)|{'evidence_manifest.json'}:
            raise RuntimeError('Archive member inventory mismatch')
        for member in members:
            if not member.isfile() or Path(member.name).is_absolute() or '..' in Path(member.name).parts:
                raise RuntimeError('Unsafe/non-regular member: '+member.name)
            stream=archive.extractfile(member)
            h=hashlib.sha256(); size=0
            while True:
                chunk=stream.read(1024*1024)
                if not chunk: break
                h.update(chunk); size+=len(chunk)
            if member.name!='evidence_manifest.json':
                item=expected[member.name]
                if size!=item['bytes'] or h.hexdigest()!=item['sha256']:
                    raise RuntimeError('Member hash/size mismatch: '+member.name)
        out.mkdir(exist_ok=True)
        extracted=[]
        for member in members:
            if Path(member.name).suffix not in ('.json','.png','.yaml','.log') and not member.name.endswith('/events.jsonl'):
                continue
            target=(out/member.name).resolve()
            if not target.is_relative_to(out.resolve()): raise RuntimeError('Extraction escaped output')
            target.parent.mkdir(parents=True,exist_ok=True)
            source=archive.extractfile(member).read()
            if target.exists() and target.read_bytes()!=source:
                raise RuntimeError('Refusing to overwrite changed evidence: '+member.name)
            if not target.exists(): target.write_bytes(source)
            extracted.append(member.name)
    verification={'passed':True,'archive_sha256':digest,'archive_bytes':ARCHIVE.stat().st_size,
                  'verified_original_files':len(expected),'verified_original_bytes':manifest['total_bytes'],
                  'all_original_member_sha256_match':True,'extracted':extracted}
    (BASE/'evidence_verification.json').write_text(json.dumps(verification,indent=2)+'\n',encoding='utf8')
    summary=json.loads((out/'green_house_trial1/analysis_summary.json').read_text())
    print(json.dumps({'verification':{k:v for k,v in verification.items() if k!='extracted'},
                      'failure_events':[{'elapsed':x.get('elapsed'),'status':x.get('status'),
                                         'reason':x.get('reason'),'detail':x.get('detail')}
                                        for x in summary['failure_status_events']],
                      'source_age_ros_s':summary['source_age_ros_s'],
                      'recorded_topic_message_counts':summary['recorded_topic_message_counts']},indent=2))


if __name__=='__main__': main()
