"""Snapshot and verify upper-application refactors offline; never starts the robot."""
from collections import Counter
from datetime import datetime, timezone
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import runpy
import shutil
import subprocess
import sys
import time
import zipfile

sys.stdout.reconfigure(encoding='utf-8')
ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / 'output/upper-review-20261113'
EVIDENCE = ROOT / 'docs/evidence/2026-11-13/upper-review-20261113'
sys.path.insert(0, str(Path(os.environ.get('TEMP', '/tmp')) / 'scara-code-audit-libs'))
import lizard


def group(path):
    if path.startswith(('app/', 'vision/', 'chess/', 'motion/')):
        return 'application_cpp'
    if path.startswith('temp/transport/'):
        return 'usb_can_adapter_excluded'
    if path.startswith('temp/yolo_live/'):
        return 'legacy_live_vision'
    if path in ('manual_control/calibrate_j1.py', 'manual_control/j1_bench.py',
                'manual_control/software_update_j1.py'):
        return 'commissioning_tools'
    if path.startswith('manual_control/'):
        return 'manual_and_demo'
    return 'support'


def sources():
    files = []
    for folder in ('app', 'vision', 'chess', 'motion', 'manual_control', 'temp/yolo_live', 'temp/transport'):
        directory = ROOT / folder
        if directory.exists():
            files.extend(p for p in directory.rglob('*')
                         if p.suffix in ('.cpp', '.hpp', '.h', '.py', '.html', '.js') and '__pycache__' not in p.parts)
    return sorted(files)


def read_source(path):
    try:
        return path.read_text(encoding='utf-8-sig'), 'utf-8-sig'
    except UnicodeDecodeError:
        return path.read_text(encoding='gb18030'), 'gb18030'


def inventory():
    items = []
    for path in sources():
        name = path.relative_to(ROOT).as_posix()
        source, encoding = read_source(path)
        if path.suffix == '.html':
            script = '\n'.join(re.findall(r'<script[^>]*>(.*?)</script>', source, re.S))
            count = lizard.analyze_file.analyze_source_code(name + '.js', script).nloc
            markup = len([line for line in re.sub(r'<script[^>]*>.*?</script>', '', source, flags=re.S).splitlines() if line.strip()])
        else:
            count = lizard.analyze_file.analyze_source_code(name, source).nloc
            markup = 0
        items.append(dict(file=name, group=group(name), nloc=count, encoding=encoding, markup_nonblank_lines=markup,
                          sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    return items


def run(command, log, timeout=120):
    result = subprocess.run(command, cwd=ROOT, capture_output=True, timeout=timeout)
    log.write_bytes(result.stdout + result.stderr)
    if result.returncode:
        print(log.read_text(encoding='utf-8', errors='replace')[-5000:])
        raise RuntimeError(f'Command failed ({result.returncode}); see {log}')
    return result


def geometry(round_dir, number):
    output = round_dir / 'geometry'
    output.mkdir(exist_ok=True)
    exe = 'build/commit-verify/scara_geometry_tool.exe'
    fixture = 'tests/fixtures/upper-app'
    cases = [('calibrate', 'measured-pairs.json', 'plane.yml'),
             ('register-arm', 'arm-touches.json', 'arm.json'),
             ('inspect', 'inspect.json', 'grid.json'),
             ('reference', 'reference.json', 'orientation.yml')]
    for command, source, target in cases:
        run([exe, command, f'{fixture}/{source}', (output / target).relative_to(ROOT).as_posix()],
            output / (command + '.log'))
    run(['build/commit-verify/scara_geometry_image_test.exe', f'{fixture}/source.png',
         (output / 'transforms').relative_to(ROOT).as_posix(), f'{fixture}/orientation.yml'],
        output / 'transforms.log')
    comparisons = []
    for file in sorted(output.rglob('*')):
        if file.is_file() and file.suffix != '.log':
            relative = file.relative_to(output)
            digest = hashlib.sha256(file.read_bytes()).hexdigest()
            if number:
                baseline = OUTPUT / 'round-00/geometry' / relative
                if baseline.read_bytes() != file.read_bytes():
                    raise AssertionError(f'Geometry output changed: {relative}')
            comparisons.append(dict(file=relative.as_posix(), sha256=digest, baseline_identical=True))
    return comparisons


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('number', type=int)
    parser.add_argument('title')
    parser.add_argument('finding')
    args = parser.parse_args()
    os.chdir(ROOT)
    OUTPUT.mkdir(parents=True, exist_ok=True)
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    output = OUTPUT / f'round-{args.number:02d}'
    output.mkdir(exist_ok=True)
    started = time.monotonic()
    before = inventory()

    checker = OUTPUT / 'check_vision.py'
    if not checker.exists():
        original = ROOT / 'output/vision-camera-open-20261113/check_round.py'
        checker.write_text(original.read_text(encoding='utf-8').replace(
            'vision-camera-open-20261113', 'upper-review-20261113'), encoding='utf-8')
    sys.argv = [str(checker), str(args.number), args.title, args.finding]
    runpy.run_path(str(checker), run_name='__main__')
    shutil.copy2(output / 'code.txt', output / 'vision-code.txt')
    shutil.copy2(output / 'code.zip', output / 'vision-code.zip')

    run([sys.executable, '-m', 'unittest', 'discover', '-s', 'tests', '-p', 'test_*.py'],
        output / 'python-tests.log')
    compared = geometry(output, args.number)
    after = inventory()
    assert before == after, 'Application sources changed during verification; rerun this round.'
    counts = Counter()
    for item in after:
        counts[item['group']] += item['nloc']
    report_path = EVIDENCE / f'round-{args.number:02d}.json'
    report = json.loads(report_path.read_text(encoding='utf-8'))
    report.update(vision_nloc=report['nloc'], source_inventory=after, group_nloc=dict(counts),
                  geometry_outputs=compared, python_tests_passed=True,
                  python_tests=(output / 'python-tests.log').read_text(encoding='utf-8', errors='replace').strip(),
                  finished_utc=datetime.now(timezone.utc).isoformat(),
                  total_elapsed_seconds=round(time.monotonic() - started, 2))
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    with (output / 'code.txt').open('w', encoding='utf-8') as view, zipfile.ZipFile(
            output / 'code.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
        for item in after:
            name = item['file']
            source, _ = read_source(ROOT / name)
            view.write('\n' + '=' * 72 + '\n' + name + '\n' + '=' * 72 + '\n' + source + '\n')
            archive.writestr(name, (ROOT / name).read_bytes())
    if args.number == 0:
        (output / 'source').mkdir(exist_ok=True)
        for item in after:
            target = output / 'source' / item['file']
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROOT / item['file'], target)
    (output / 'changes.diff').write_bytes(subprocess.check_output(['git', 'diff', '--',
        'app', 'vision', 'chess', 'motion', 'manual_control', 'temp/yolo_live', 'tests', 'CMakeLists.txt']))
    print(json.dumps(dict(round=args.number, group_nloc=dict(counts), geometry_outputs=len(compared),
                         full_source=(output / 'code.txt').relative_to(ROOT).as_posix(),
                         elapsed_seconds=report['total_elapsed_seconds']), ensure_ascii=False))


if __name__ == '__main__':
    main()
