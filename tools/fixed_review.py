"""Review the fixed-board simplification against its frozen offline baseline."""
from collections import Counter
from datetime import datetime, timezone
import argparse
import hashlib
import json
import os
import re
from pathlib import Path
import runpy
import shutil
import subprocess
import sys
import zipfile
import upper_review as upper

ROOT = upper.ROOT
OUTPUT = ROOT / 'output/fixed-minimal-20261113'
EVIDENCE = ROOT / 'docs/evidence/2026-11-13/fixed-minimal-20261113'


def geometry(output, number):
    directory = output / 'geometry'
    directory.mkdir(exist_ok=True)
    cases = [('calibrate', 'tests/fixtures/upper-app/measured-pairs.json', 'plane.yml'),
             ('inspect', 'tests/fixtures/fixed-app/setup.json', 'grid.json')]
    for command, source, name in cases:
        upper.run(['build/commit-verify/scara_geometry_tool.exe', command, source,
                   str((directory / name).relative_to(ROOT))], directory / (command + '.log'))
    files = []
    for path in sorted(directory.iterdir()):
        if path.suffix != '.log':
            if number:
                assert path.read_bytes() == (OUTPUT / 'round-00/geometry' / path.name).read_bytes(), path.name
            files.append(dict(file=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    return files


def main():
    global OUTPUT, EVIDENCE
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('number', type=int)
    parser.add_argument('title')
    parser.add_argument('finding')
    parser.add_argument('--series', help='Independent review directory, ending in YYYYMMDD')
    args = parser.parse_args()
    if args.series:
        if not re.fullmatch(r'[a-z0-9-]+-\d{8}', args.series):
            parser.error('Invalid review directory name')
        date = args.series[-8:]
        OUTPUT = ROOT / 'output' / args.series
        EVIDENCE = ROOT / 'docs/evidence' / (date[:4] + '-' + date[4:6] + '-' + date[6:]) / args.series
    os.chdir(ROOT)
    output = OUTPUT / f'round-{args.number:02d}'
    output.mkdir(parents=True, exist_ok=True)
    before = upper.inventory()
    checker = OUTPUT / 'check_vision.py'
    sys.argv = [str(checker), str(args.number), args.title, args.finding]
    runpy.run_path(str(checker), run_name='__main__')
    shutil.copy2(output / 'code.txt', output / 'vision-code.txt')
    shutil.copy2(output / 'code.zip', output / 'vision-code.zip')
    upper.run([sys.executable, '-m', 'unittest', 'discover', '-s', 'tests', '-p', 'test_*.py'],
              output / 'python-tests.log')
    geometry_outputs = geometry(output, args.number)
    after = upper.inventory()
    assert before == after, 'Sources changed during review.'
    counts = Counter()
    for entry in after:
        counts[entry['group']] += entry['nloc']
    report_path = EVIDENCE / f'round-{args.number:02d}.json'
    report = json.loads(report_path.read_text(encoding='utf-8'))
    report.update(source_inventory=after, group_nloc=dict(counts), geometry_outputs=geometry_outputs,
                  python_tests=(output / 'python-tests.log').read_text(encoding='utf-8', errors='replace'),
                  finished_utc=datetime.now(timezone.utc).isoformat())
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    with (output / 'code.txt').open('w', encoding='utf-8') as view, zipfile.ZipFile(
            output / 'code.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
        for entry in after:
            name = entry['file']
            source, _ = upper.read_source(ROOT / name)
            view.write('\n' + '=' * 72 + '\n' + name + '\n' + '=' * 72 + '\n' + source + '\n')
            archive.writestr(name, (ROOT / name).read_bytes())
    with (output / 'cpp-code.txt').open('w', encoding='utf-8') as view, zipfile.ZipFile(
            output / 'cpp-code.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
        for entry in after:
            if entry['group'] == 'application_cpp':
                name = entry['file']
                source, _ = upper.read_source(ROOT / name)
                view.write('\n' + '=' * 72 + '\n' + name + '\n' + '=' * 72 + '\n' + source + '\n')
                archive.writestr(name, (ROOT / name).read_bytes())
    (output / 'changes.diff').write_bytes(subprocess.check_output(['git', 'diff', '--',
        'app', 'vision', 'chess', 'motion', 'manual_control', 'temp/yolo_live', 'tests', 'CMakeLists.txt']))
    print(json.dumps(dict(round=args.number, group_nloc=dict(counts), geometry_files=len(geometry_outputs)),
                     ensure_ascii=False))


if __name__ == '__main__':
    main()
