"""Run the actual match entry with scripted scenes, engine answers and direct actuator results."""
import copy
import json
from pathlib import Path
import shutil
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
                assert action in (2,3)
                step = dict(action='discard' if action == 3 else 'release', duration_ms=duration)
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


def run(name, scenario, history=None, exit_code=0, calls=3, settings=None, placement_offsets=None):
    global passed
    with tempfile.TemporaryDirectory(prefix='scara_match_') as directory:
        work = Path(directory)
        config = copy.deepcopy(TEMPLATE)
        for key in ('layout_file', 'arm_file', 'heights_file'):
            source = (ROOT / 'tests/fixtures/game-app' / config[key]).resolve()
            fixture = work / f'{key}.json'
            shutil.copyfile(source, fixture)
            config[key] = str(fixture)
        if placement_offsets is not None:
            layout_file = Path(config['layout_file'])
            layout = json.loads(layout_file.read_text(encoding='utf-8'))
            layout['placement_offsets_mm'] = placement_offsets
            layout_file.write_text(json.dumps(layout), encoding='utf-8')
        config['work_directory'] = str(work / 'runtime')
        config['simulation_file'] = str(work / 'scenario.json')
        config.update(settings or {})
        (work / 'scenario.json').write_text(json.dumps(scenario), encoding='utf-8')
        (work / 'config.json').write_text(json.dumps(config), encoding='utf-8')
        result = subprocess.run([str(EXE), str(work / 'config.json')], cwd=ROOT,
                                capture_output=True, text=True, encoding='utf-8', errors='replace', timeout=8)
        assert result.returncode == exit_code, (name, result.returncode, result.stdout[-2000:], result.stderr)
        line = next(line for line in result.stdout.splitlines() if line.startswith('SIM_RESULT\t'))
        _, phase, outcome, pending, valid, frame, moves, board, seen, message = line.split('\t', 9)
        assert len(board) == len(seen) == 90 and not (work/'runtime').exists()
        rows = lambda value: [value[i:i+9] for i in range(0,90,9)]
        state = dict(simulated=1, phase=phase, result=int(outcome), pending=int(pending), observation_valid=int(valid),
                     frame_id=frame, history=moves, board=rows(board), observed=rows(seen), error=message)
        assert state['simulated'] == 1, name
        assert state['history'] == (BASE['expected_history'] if history is None else history), (name, state)
        executions = read_calls(result.stdout)
        assert len(executions) == calls, (name, len(executions), result.stdout)
        assert not list((work / 'runtime').glob('motion-*.json')), 'Retired motion protocol is still active'
        assert not list((work / 'runtime').glob('request-*.json')), 'Simulation still uses request files'
        assert all(execution['simulated'] == 1 for execution in executions)
        assert [step['action'] for step in executions[0]['steps']] == ['release', 'move', 'move']
        assert executions[0]['steps'][1]['axes'] == 1
        heights = json.loads(Path(config['heights_file']).read_text(encoding='utf-8'))['z']
        park = [heights['travel_deg'], *config['park_joints_deg']]
        for execution in executions[1:]:
            assert execution['steps'][-1] == dict(action='move', axes=7, target_deg=park)
            assert execution['end_joint_deg'] == park
            if execution['purpose'] == 'prepare':
                assert [s['action'] for s in execution['steps']] == ['release', 'move', 'move']
                assert execution['steps'][1]['axes'] == 1 and execution['steps'][1]['target_deg'][0] == heights['travel_deg']
                continue
            for i, step in enumerate(execution['steps']):
                if step['action'] == 'release':
                    assert execution['steps'][i + 1]['action'] == 'move', name
                    assert execution['steps'][i + 1]['target_deg'][0] == heights.get('clearance_deg', heights['travel_deg'])
        passed += 1
        print('PASS', name)
        return state, executions, result.stdout


state, executions, output = run('two rounds with robot capture', BASE)
assert state['board'] == BASE['expected_board'] and state['pending'] == 0
assert [len(execution['steps']) for execution in executions] == [3, 9, 15]
assert executions[2]['steps'][5]['action'] == 'discard'

_, compensated, _ = run('ordinary pickup, capture and placement use the same grid targets', BASE,
                         placement_offsets=[[0, 5, 2., -1.5], [0, 6, 2., -1.5], [0, 9, 2., -1.5]])
ordinary, capture = (e['steps'] for e in compensated[1:])
assert ordinary[4]['target_deg'][1:] == capture[1]['target_deg'][1:] == capture[10]['target_deg'][1:]
assert ordinary[1] != executions[1]['steps'][1]
assert capture[7] != executions[2]['steps'][7]

case = copy.deepcopy(BASE)
after_human_capture = case['commands'][3]['board']
case['commands'][2:4] = [
    dict(phase='human', command='a3a2'),
    dict(phase='human', command='6,0 5,0'),
    dict(phase='human', command='a4a5'),
]
case['executions'].insert(1, dict(purpose='move', move='a3a4', success=1, board=AFTER_HUMAN))
case['executions'].insert(3, dict(purpose='move', move='a4a5', success=1, board=after_human_capture))
state, executions, output = run('panel plays both sides, rejects illegal selection and handles a capture', case, calls=5)
assert 'SIM_NOTICE illegal_move' in output and state['pending'] == 0
assert [execution['move'] for execution in executions[1:]] == ['a3a4', 'a6a5', 'a4a5', 'a9a5']
assert len(executions[3]['steps']) == 15

# 事件回调只显示；相机输入按场景模拟人类恢复，恢复前不能下发新的搬棋动作。
case = copy.deepcopy(BASE)
swapped = AFTER_HUMAN.copy()
rows = [list(row) for row in swapped]
rows[7][1], rows[9][1] = rows[9][1], rows[7][1]
swapped = [''.join(row) for row in rows]
case['queries'][1]['board_after_think'] = swapped
case['restorations'] = [dict(phase='restore_board', after_frames=6, board=AFTER_HUMAN)]
_, _, output = run('swap while thinking restores before moving', case)
assert '(7,1)' in output and '(9,1)' in output
event_at = output.index('SIM_NOTICE restore_board')
assert output.index('SIM_NOTICE restored', event_at) < output.index('SIM_CALL move a6a5', event_at)

case = copy.deepcopy(BASE)
missing = AFTER_HUMAN.copy()
row = list(missing[9]); row[4] = '.'; missing[9] = ''.join(row)
case['queries'][1]['board_after_think'] = missing
case['restorations'] = [dict(phase='restore_board', after_frames=3, board=AFTER_HUMAN)]
_, _, output = run('missing king is a board event rather than victory', case)
assert '(9,4)' in output

case = copy.deepcopy(BASE)
case['queries'][1]['board_after_think'] = swapped
case['cancel_after_frames'] = 30
state, _, _ = run('unrestored board never starts robot movement', case, 'a3a4', calls=1)
assert state['pending'] == 0 and state['phase'] == 'stopped'

case = copy.deepcopy(BASE)
case['commands'][2]['board'] = swapped
case['commands'].insert(3,dict(phase='human',command='ready',board=AFTER_HUMAN))
case['restorations']=[dict(phase='illegal_move',after_frames=3,board=INITIAL)]
_, _, output = run('illegal human change restores before next attempt',case)
assert 'SIM_NOTICE illegal_move' in output

case = copy.deepcopy(BASE)
retry = copy.deepcopy(case['executions'][1])
case['executions'][1].update(success=0, board=AFTER_HUMAN)
case['executions'][2:2] = [dict(purpose='prepare', move='', success=1), retry]
case['commands'].insert(3, dict(phase='hardware_recovery', command='ready', board=AFTER_HUMAN))
_, _, output = run('motion failure still requires hardware recovery', case, calls=5)
assert 'SIM_COMMAND hardware_recovery ready' in output
assert output.count('SIM_CALL move a6a5') == 2

case = copy.deepcopy(BASE)
retry = copy.deepcopy(case['executions'][1])
case['executions'][1]['board'] = AFTER_HUMAN
case['executions'].insert(2, retry)
case['commands'].insert(3, dict(phase='placement_recovery', command='ready', board=AFTER_HUMAN))
_, _, output = run('restored board retries without re-parking', case, calls=4)
assert output.count('SIM_CALL prepare') == 1 and output.count('SIM_CALL move a6a5') == 2

case = copy.deepcopy(BASE)
case['executions'][1]['board'] = AFTER_HUMAN
case['commands'].insert(3, dict(phase='placement_recovery', command='ready', board=AFTER_ROBOT))
_, _, output = run('visually corrected placement commits without repeating movement', case)
assert 'SIM_COMMAND placement_recovery ready' in output
assert 'SIM_COMMAND hardware_recovery' not in output and output.count('SIM_CALL move a6a5') == 1

case = copy.deepcopy(BASE)
case['executions'][1]['off_grid'] = 'i0'
case['restorations'] = [dict(phase='unclear', after_frames=1, board=AFTER_ROBOT)]
_, _, output = run('off-grid rook recheck continues once centered', case)
assert '(9,8)' in output and output.count('SIM_CALL move a6a5') == 1

case.pop('restorations')
case['cancel_after_frames'] = 18
state, _, output = run('persistent off-grid rook cannot commit or restart movement', case, 'a3a4', calls=2)
assert state['pending'] == 1 and state['observation_valid'] == 0 and state['observed'][9][8] == '?'

case=copy.deepcopy(BASE)
case['executions'][1].update(success=0,board=AFTER_HUMAN)
case['commands'].insert(3,dict(phase='hardware_recovery',command='quit'))
state,_,_=run('quit preserves the last confirmed history',case,'a3a4',calls=2)
assert state['pending']==1

case=copy.deepcopy(BASE)
case['queries'][1]['fail_once']=1
case['commands'].insert(3,dict(phase='engine_recovery',command='ready'))
_,_,output=run('engine failure is an event and retries the same turn',case)
assert 'SIM_NOTICE engine_recovery' in output

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

# A saved position is confirmed against the camera without clearing its turn or history.
case = copy.deepcopy(BASE)
case['resume_moves'] = ['a3a4']
case['initial_board'] = AFTER_HUMAN
case['commands'] = case['commands'][:2] + [dict(phase='human', command='quit')]
case['executions'] = case['executions'][:2]
case['queries'] = [dict(history='a3a4', legal=['a6a5'], choice='a6a5'),
                   dict(history='a3a4 a6a5', legal=['a4a5'])]
state, _, _ = run('resume preserves black turn and appends the reply', case, 'a3a4 a6a5', calls=2)
assert state['board'] == AFTER_ROBOT
case['initial_board'] = INITIAL
state, _, _ = run('resume mismatch preserves history and sends no chess move', case, 'a3a4', exit_code=1, calls=1)
assert state['board'] == AFTER_HUMAN and state['pending'] == 0 and state['phase'] == 'error'

# Three distinct consecutive frames belong to observation, independently of the fixed grid.
for name, ids in [('repeated camera frames', ['1','1','2','2']),
                  ('missing frame resets confirmation', ['1','2','','3','4'])]:
    case = copy.deepcopy(BASE)
    case['frame_ids'] = ids
    case['cancel_after_frames'] = len(ids)
    case['commands'] = case['commands'][:1]
    case['executions'] = case['executions'][:1]
    state, _, output = run(name, case, '', calls=1)
    assert state['observation_valid'] == 0 and 'SIM_COMMAND confirm_start' not in output

print(f'PASS {passed} actual-main scenarios; scripted perception/engine/execution, no hardware.')
