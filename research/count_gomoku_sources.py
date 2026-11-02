"""Count downloaded reference sources without importing or running them.

Usage: python count_gomoku_sources.py TEMP_DOWNLOAD_ROOT OUTPUT_JSON
Counts physical and nonblank lines, both INCLUDING comments. This is not SLOC.
"""
import hashlib
import json
from pathlib import Path
import sys

SOURCE_EXTENSIONS = {
    '.py', '.c', '.h', '.cc', '.cpp', '.hpp', '.ino', '.pde',
    '.m', '.js', '.ts', '.html', '.css', '.cs',
}
SKIP_PARTS = {'.git', '__pycache__', 'node_modules', 'Debug', 'Release', 'build'}


def count(root):
    results = []
    for folder in sorted(root.iterdir()):
        if not folder.is_dir() or '__' not in folder.name:
            continue
        records = []
        for path in sorted(folder.rglob('*')):
            relative = path.relative_to(folder)
            if not path.is_file() or set(relative.parts) & SKIP_PARTS:
                continue
            suffix = path.suffix.lower()
            if suffix not in SOURCE_EXTENSIONS and suffix != '.ipynb':
                continue
            data = path.read_bytes()
            if suffix in SOURCE_EXTENSIONS:
                lines = data.decode('utf8', errors='replace').splitlines()
                kind = 'source'
            elif suffix == '.ipynb':
                notebook = json.loads(data.decode('utf-8-sig'))
                lines = []
                for cell in notebook.get('cells', []):
                    if cell.get('cell_type') == 'code':
                        lines.extend(''.join(cell['source']).splitlines())
                kind = 'notebook-code-cells'
            else:
                continue
            records.append({
                'path': relative.as_posix(),
                'kind': kind,
                'physical_lines': len(lines),
                'nonblank_lines_including_comments': sum(bool(line.strip()) for line in lines),
                'file_bytes': len(data),
                'sha256': hashlib.sha256(data).hexdigest(),
            })
        results.append({
            'repo': folder.name.replace('__', '/'),
            'counted_files': len(records),
            'physical_lines': sum(row['physical_lines'] for row in records),
            'nonblank_lines_including_comments': sum(row['nonblank_lines_including_comments'] for row in records),
            'files': records,
        })
    return results


if __name__ == '__main__':
    result = count(Path(sys.argv[1]))
    Path(sys.argv[2]).write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf8')
    print(f"Saved {len(result)} repository counts to {sys.argv[2]}")
