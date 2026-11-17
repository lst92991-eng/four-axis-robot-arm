"""Run the actual match entry with scripted scenes, engine answers and direct actuator results."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
EXE = Path(sys.argv[1]).resolve()
BASE = json.loads((ROOT / 'tests/fixtures/game-app/two-rounds.json').read_text(encoding='utf-8'))
TEMPLATE = json.loads((ROOT / 'tests/fixtures/game-app/simulation.json').read_text(encoding='utf-8'))
INITIAL = BASE['initial_board']
AFTER_HUMAN = BASE['commands'][2]['board']
AFTER_ROBOT = BASE['executions'][1]['board']
passed = 0


def read_calls(output):
    calls = []
    current = None
    for line in output.splitlines():
        fields = line.split()
        if line.startswith('SIM_CALL '):
            assert current is None, 'Overlapping direct calls'
            current = dict(simulated=1, purpose=fields[1], move='' if fields[2] == '-' else fields[2], steps=[])
        elif line.startswith('SIM_STEP '):
            assert current is not None
            action = int(fields[1])
            axes = int(fields[2])
            target = list(map(float, fields[3:6]))
            duration = int(fields[6])
            if action == 0:
                step = dict(action='hold')
            elif action == 1:
                step = dict(action='move', axes=axes, target_deg=target)
            else:
                assert action == 2
                step = dict(action='release', duration_ms=duration)
            current['steps'].append(step)
        elif line.startswith('SIM_POSE '):
            current['end_joint_deg'] = list(map(float, fields[1:4]))
            current['reference_count'] = int(fields[4])
        elif line.startswith('SIM_RETURN '):
            assert current is not None and fields[1] in ('0', '1')
            calls.append(current)
            current = None
    assert current is None, 'A direct call did not return'
    return calls


def run(name, scenario, history=None, exit_code=0, calls=3, settings=None):
    global passed
    with tempfile.TemporaryDirectory(prefix='scara_对弈_') as directory:
        work = Path(directory)
        config = copy.deepcopy(TEMPLATE)
        for key in ('layout_file', 'arm_file', 'heights_file'):
            config[key] = str((ROOT / 'tests/fixtures/game-app' / config[key]).resolve())
        config['work_directory'] = str(work / 'runtime')
        config['simulation_file'] = str(work / 'scenario.json')
        config.update(settings or {})
        (work / 'scenario.json').write_text(json.dumps(scenario), encoding='utf-8')
        (work / 'config.json').write_text(json.dumps(config), encoding='utf-8')
        result = subprocess.run([str(EXE), str(work / 'config.json')], cwd=ROOT,
                                capture_output=True, text=True, encoding='utf-8', errors='replace', timeout=8)
        assert result.returncode == exit_code, (name, result.returncode, result.stdout[-2000:], result.stderr)
        state = json.loads((work / 'runtime/status.json').read_text(encoding='utf-8'))
        assert state['simulated'] == 1, name
        assert state['history'] == (BASE['expected_history'] if history is None else history), (name, state)
        executions = read_calls(result.stdout)
        assert len(executions) == calls, (name, len(executions), result.stdout)
        assert not list((work / 'runtime').glob('motion-*.json')), 'Retired motion protocol is still active'
        assert not list((work / 'runtime').glob('request-*.json')), 'Simulation still uses request files'
        assert all(execution['simulated'] == 1 for execution in executions)
        assert [step['action'] for step in executions[0]['steps']] == ['hold', 'move', 'move']
        assert executions[0]['steps'][1]['axes'] == 1
        heights = json.loads(Path(config['heights_file']).read_text(encoding='utf-8'))['z']
        park = [heights['travel_deg'], *config['park_joints_deg']]
        for execution in executions[1:]:
            assert execution['steps'][-1] == dict(action='move', axes=7, target_deg=park)
            assert execution['end_joint_deg'] == park
            for i, step in enumerate(execution['steps']):
                if step['action'] == 'release':
                    assert execution['steps'][i + 1]['action'] == 'move', name
                    assert execution['steps'][i + 1]['target_deg'][0] == heights.get('clearance_deg', heights['travel_deg'])
        passed += 1
        print('PASS', name)
        return state, executions, result.stdout


state, executions, output = run('two rounds with robot capture', BASE)
assert state['board'] == BASE['expected_board'] and state['pending'] == 0
assert [len(execution['steps']) for execution in executions] == [3, 9, 17]

# 事件回调只显示；相机输入按场景模拟人类恢复，恢复前不能下发新的搬棋动作。
case = copy.deepcopy(BASE)
swapped = AFTER_HUMAN.copy()
rows = [list(row) for row in swapped]
rows[7][1], rows[9][1] = rows[9][1], rows[7][1]
swapped = [''.join(row) for row in rows]
case['queries'][1]['board_after_think'] = swapped
case['restorations'] = [dict(event=1, after_frames=6, board=AFTER_HUMAN)]
_, _, output = run('swap while thinking restores before moving', case)
assert '棋子位置互换' in output and '红马' in output and '红炮' in output
event_at = output.index('SIM_EVENT 1')
assert output.index('SIM_EVENT 7', event_at) < output.index('SIM_CALL move a6a5', event_at)

case = copy.deepcopy(BASE)
missing = AFTER_HUMAN.copy()
row = list(missing[9]); row[4] = '.'; missing[9] = ''.join(row)
case['queries'][1]['board_after_think'] = missing
case['restorations'] = [dict(event=1, after_frames=3, board=AFTER_HUMAN)]
_, _, output = run('missing king is a board event rather than victory', case)
assert '棋子缺失' in output and 'e0 红帅' in output

case = copy.deepcopy(BASE)
case['queries'][1]['board_after_think'] = swapped
case['cancel_after_frames'] = 30
state, _, _ = run('unrestored board never starts robot movement', case, 'a3a4', calls=1)
assert state['pending'] == 0 and state['phase'] == 'stopped'

case = copy.deepcopy(BASE)
case['commands'][2]['board'] = swapped
case['commands'].insert(3,dict(phase='human',command='ready',board=AFTER_HUMAN))
case['restorations']=[dict(event=2,after_frames=3,board=INITIAL)]
_, _, output = run('illegal human change restores before next attempt',case)
assert 'SIM_EVENT 2' in output

for failure in ('returned_failure','wrong_placement'):
    case=copy.deepcopy(BASE)
    retry=copy.deepcopy(case['executions'][1])
    case['executions'][1]['board']=AFTER_HUMAN
    case['executions'][1]['success']=0 if failure=='returned_failure' else 1
    case['executions'][2:2]=[dict(purpose='prepare',move='',success=1),retry]
    case['commands'].insert(3,dict(phase='hardware_recovery',command='ready',board=AFTER_HUMAN))
    _, _, output=run('hardware confirmation and restore: '+failure,case,calls=5)
    assert 'SIM_COMMAND hardware_recovery ready' in output
    assert output.count('SIM_CALL move a6a5')==2

case=copy.deepcopy(BASE)
case['executions'][1].update(success=0,board=AFTER_HUMAN)
case['commands'].insert(3,dict(phase='hardware_recovery',command='quit'))
state,_,_=run('quit preserves the last confirmed history',case,'a3a4',calls=2)
assert state['pending']==1

case=copy.deepcopy(BASE)
case['queries'][1]['fail_once']=1
case['commands'].insert(3,dict(phase='engine_recovery',command='ready'))
_,_,output=run('engine failure is an event and retries the same turn',case)
assert 'SIM_EVENT 5' in output

case=copy.deepcopy(BASE)
case['queries'][1]['fail']=1
case['commands'].insert(3,dict(phase='engine_recovery',command='quit'))
state,_,_=run('engine cannot be mistaken for an empty legal list',case,'a3a4',calls=1)
assert state['result']==0

case=copy.deepcopy(BASE)
case['queries'][1]['choice']='b9b8'
state,_,_=run('illegal engine choice issues no movement',case,'a3a4',1,1)
assert state['pending']==0

case=copy.deepcopy(BASE)
case['queries'][0]['legal']=[]
state,_,_=run('terminal signal',case,'',calls=1)
assert state['result']==2

case=copy.deepcopy(BASE)
case['executions'][0]['success']=0
case['commands'].insert(1,dict(phase='hardware_recovery',command='quit'))
run('preparation failure cannot start a game',case,'',calls=1)

case=copy.deepcopy(BASE)
case['commands'][2]=dict(phase='human',command='resign')
state,_,_=run('human resignation',case,'',calls=1)
assert state['result']==2

case=copy.deepcopy(BASE)
case['commands']=case['commands'][:2]+[dict(phase='human',command='quit')]
case['queries'][0]['choice']='a3a4';case['queries'][1]['choice']=''
case['executions']=[case['executions'][0],dict(purpose='move',move='a3a4',success=1,board=AFTER_HUMAN)]
run('robot can play red first',case,'a3a4',calls=2,settings={'human_red':0})

case=copy.deepcopy(BASE)
case['commands']=[dict(phase='prepare',command='ready'),dict(phase='confirm_start',command='ready',board=AFTER_HUMAN),dict(phase='confirm_start',command='ready')]
case['queries']=[dict(history='',legal=[])]
state,_,output=run('changed initial board is confirmed again',case,'',calls=1)
assert output.count('SIM_COMMAND confirm_start ready')==2

# Two legal knight cycles return to the initial position three times. The app
# must request human adjudication, not announce a draw or a winner.
red_knight = INITIAL.copy()
red_knight[9] = 'R.BAKABNR'
red_knight[7] = '.CN....C.'
both_knights = red_knight.copy()
both_knights[0] = 'r.bakabnr'
both_knights[2] = '.cn....c.'
black_knight = both_knights.copy()
black_knight[9] = INITIAL[9]
black_knight[7] = INITIAL[7]
moves = ['b0c2', 'b9c7', 'c2b0', 'c7b9'] * 2
boards = [red_knight, both_knights, black_knight, INITIAL] * 2
case = copy.deepcopy(BASE)
case['commands'] = case['commands'][:2]
case['executions'] = case['executions'][:1]
case['queries'] = []
for index, move in enumerate(moves):
    query = dict(history=' '.join(moves[:index]), legal=[move])
    if index % 2:
        query['choice'] = move
        case['executions'].append(dict(purpose='move', move=move, success=1, board=boards[index]))
    else:
        case['commands'].append(dict(phase='human', command='ready', board=boards[index]))
    case['queries'].append(query)
case['queries'].append(dict(history=' '.join(moves), legal=['b0c2']))
state, _, _ = run('repetition requires adjudication', case, ' '.join(moves), calls=5)
assert state['result'] == 4 and state['phase'] == 'review_required'

print(f'PASS {passed} actual-main scenarios; scripted perception/engine/execution, no hardware.')
