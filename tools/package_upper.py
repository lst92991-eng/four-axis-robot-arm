"""Bundle the audited upper application and its offline fixtures, without running hardware."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--review', choices=('upper-review-20261113', 'fixed-minimal-20261113', 'game-main-20261114',
                                            'game-lesson-20261114', 'game-style-20261114'),
                        default='fixed-minimal-20261113')
    parser.add_argument('--round', type=int, required=True)
    args = parser.parse_args()
    review = Path('output') / args.review
    date = args.review[-8:]
    evidence = Path('docs/evidence') / (date[:4] + '-' + date[4:6] + '-' + date[6:]) / args.review
    current = review / f'round-{args.round:02d}'
    report = json.loads((ROOT / evidence / f'round-{args.round:02d}.json').read_text(encoding='utf-8'))
    files = set()
    for entry in report['source_inventory']:
        source = ROOT / entry['file']
        assert hashlib.sha256(source.read_bytes()).hexdigest() == entry['sha256'], entry['file']
        files.add(Path(entry['file']))
    files.update(Path(name) for name in (
        'README.md', 'AGENTS.md', 'CMakeLists.txt', '.vscode/settings.json', 'cmake/rv1106.cmake',
        'tools/run-vision.sh', 'tools/upper_review.py', 'tools/fixed_review.py', 'tools/package_upper.py',
        'docs/项目收敛与下棋闭环_2026-11-09.md', 'docs/棋局与视觉接口_2026-11-09.md'))
    files.add(review / 'check_vision.py')
    if args.review.startswith('game-'):
        files.update(Path(name) for name in ('tools/game_review.py', 'tools/run-game.sh', 'tools/game-preview.html'))
    for folder in (Path('config'), Path('tests'), evidence, review/'round-00/replay',
                   review/'round-00/geometry', current/'replay', current/'geometry'):
        for path in (ROOT / folder).rglob('*'):
            if path.is_file() and '__pycache__' not in path.parts and path.suffix != '.pyc':
                if path.name != 'package-manifest.json':
                    files.add(path.relative_to(ROOT))
    files.update(current / name for name in (
        'code.txt', 'vision-code.txt', 'build-test.log', 'python-tests.log', 'accuracy.log'))
    if args.review.startswith(('fixed-', 'game-')):
        files.update(current / name for name in ('cpp-code.txt', 'cpp-code.zip'))
    inventory = []
    for path in sorted(files):
        data = (ROOT / path).read_bytes()
        inventory.append(dict(file=path.as_posix(), bytes=len(data), sha256=hashlib.sha256(data).hexdigest()))
    manifest = dict(
        generated_utc=datetime.now(timezone.utc).isoformat(), review=args.review, tested_round=args.round,
        group_nloc=report['group_nloc'],
        scope='Own upper-application source, build files, offline tests, fixtures and frozen comparison outputs.',
        excluded='Motor firmware, vendor SDK/runtime, engine binaries, model weights, deployment and live machine state.',
        build='See README: C++17, OpenCV 5.0.0, Python test dependencies; optional actual RKNN header for ABI tests.',
        reproduction='The review scripts use the existing local Windows environment. Configure build/commit-verify and provide the original v3 ONNX before a new round.',
        model=dict(onnx_path='temp/model-v3-release/extracted/xiangqi14-local-v3.onnx',
                   onnx_sha256='fdf31b53da7bc2350936bb51fb6e53ddf0d5958aa36830a6493a187332458047',
                   rknn_sha256='ce9c91f4668cb0ec58f836432546b7eb2aebfc675ba696ce9788968560dcbdd2'),
        files=inventory)
    name = 'game-application-source.zip' if args.review.startswith('game-') else (
        'fixed-application-source.zip' if args.review.startswith('fixed-') else 'upper-application-source.zip')
    bundle = ROOT / review / name
    with zipfile.ZipFile(bundle, 'w', zipfile.ZIP_DEFLATED) as archive:
        for entry in inventory:
            archive.writestr(entry['file'], (ROOT / entry['file']).read_bytes())
        archive.writestr('SOURCE_PACKAGE.json', json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    with zipfile.ZipFile(bundle) as archive:
        assert archive.testzip() is None
        for entry in inventory:
            assert hashlib.sha256(archive.read(entry['file'])).hexdigest() == entry['sha256'], entry['file']
    manifest['archive'] = dict(file=bundle.relative_to(ROOT).as_posix(), bytes=bundle.stat().st_size,
                               sha256=hashlib.sha256(bundle.read_bytes()).hexdigest())
    (ROOT / evidence / 'package-manifest.json').write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(dict(files=len(inventory), archive=manifest['archive']), ensure_ascii=False))


if __name__ == '__main__':
    main()
