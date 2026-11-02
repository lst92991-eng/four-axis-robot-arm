from pathlib import Path
import subprocess, json, hashlib
root=Path(r'C:/Users/lst/AppData/Local/Temp/scara-research-platforms')
out=Path(r'C:/Users/lst/Desktop/scara/research')
out.mkdir(exist_ok=True)
EXT={'.c','.h','.cpp','.hpp','.cc','.ino','.cs','.ts','.tsx','.js','.jsx','.mjs','.css','.py','.pde'}
def group(repo,p):
    if repo=='ayuugoyal-scara':
        if p.startswith('frontend/components/ui/'): return 'ui_component_scaffolding'
        if p.startswith('arduino_scara/'): return 'firmware'
        if p.startswith('backend/src/'): return 'backend_application'
        if p.startswith('frontend/') and not p.endswith(('config.mjs','config.ts')): return 'frontend_application_and_helpers'
        return 'build_config_code'
    if repo=='Marcel3245-SCARA-robot-V2':
        if '.Designer.' in p: return 'generated_winforms_designer'
        return 'firmware' if p.startswith('main/') else 'host_application'
    if repo=='techyg-Pybot': return 'duplicated_subdirectory_copy' if '/' in p else 'canonical_firmware'
    if repo=='jared-hughes-grbl-scara':
        if p.startswith('doc/'): return 'example_host_scripts'
        if '/examples/' in p: return 'example_upload_sketch'
        return 'firmware_including_inherited_grbl'
    if repo in {'bdring-FluidNC','bdring-FluidNC-Scara','derFrickler-FluidNC_scara'}:
        if p.startswith('FluidNC/src/'): return 'firmware_src_including_any_embedded_libraries'
        if p.startswith('lib/'): return 'vendored_libs'
        if p.startswith('FluidNC/test/') or p.startswith('X86TestSupport/') or 'test' in p.lower(): return 'tests_and_test_support'
        return 'tools_examples_other'
    if repo=='MarlinFirmware-Marlin':
        if p.startswith('Marlin/src/lcd/') or p.startswith('Marlin/src/pins/'): return 'lcd_assets_board_pins'
        if p.startswith('Marlin/src/HAL/') or p.startswith('Marlin/src/libs/'): return 'hal_and_embedded_libraries'
        if p.startswith('Marlin/src/'): return 'firmware_other_src'
        if p.startswith('Marlin/'): return 'firmware_config_and_entry'
        return 'tools_tests_build_support'
results=[]
for repo in sorted(root.iterdir()):
    if not (repo/'.git').is_dir(): continue
    paths=subprocess.check_output(['git','-C',str(repo),'ls-files','-z']).decode('utf8').split('\0')
    rows=[]
    for p in paths:
        if not p or Path(p).suffix.lower() not in EXT: continue
        data=(repo/p).read_bytes()
        lines=data.decode('utf8',errors='replace').splitlines()
        rows.append(dict(path=p,group=group(repo.name,p),physical=len(lines),nonblank=sum(bool(line.strip()) for line in lines),bytes=len(data),sha256=hashlib.sha256(data).hexdigest()))
    sums={}
    for row in rows:
        s=sums.setdefault(row['group'],dict(files=0,physical=0,nonblank=0,bytes=0))
        s['files']+=1
        for key in ('physical','nonblank','bytes'): s[key]+=row[key]
    result=dict(repository=repo.name,commit=subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD']).decode().strip(),commit_date=subprocess.check_output(['git','-C',str(repo),'log','-1','--format=%cs']).decode().strip(),groups=sums,files=rows)
    results.append(result)
(out/'scara-platforms-metrics.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf8')
print(f"Saved {len(results)} repository counts to {out/'scara-platforms-metrics.json'}")

