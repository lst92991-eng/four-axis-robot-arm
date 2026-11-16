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
            j2, j3, z, speed = map(float, fields[2:6])
            duration = int(fields[6])
            if action == 0:
                step = dict(action='hold')
            elif action == 1:
                step = dict(action='xy', j2_deg=j2, j3_motor_deg=j3)
            elif action == 2:
                step = dict(action='z', j1_deg=z, speed_deg_s=speed)
            else:
                assert action == 3
                step = dict(action='release', duration_ms=duration)
            current['steps'].append(step)
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
        assert [step['action'] for step in executions[0]['steps']] == ['hold', 'z', 'xy']
        for execution in executions[1:]:
            assert execution['steps'][-1] == dict(action='xy', j2_deg=config['park_joints_deg'][0],
                                              j3_motor_deg=config['park_joints_deg'][1])
            for i, step in enumerate(execution['steps']):
                if step['action'] == 'release':
                    assert execution['steps'][i + 1]['action'] == 'z', name
        passed += 1
        print('PASS', name)
        return state, executions, result.stdout


state, executions, output = run('two rounds with robot capture', BASE)
assert state['board'] == BASE['expected_board'] and state['pending'] == 0
assert [len(execution['steps']) for execution in executions] == [3, 9, 17]

case = copy.deepcopy(BASE)
case['executions'][1]['board'] = AFTER_HUMAN
case['commands'].insert(3, dict(phase='recovery', command='retry', board=AFTER_ROBOT))
state, _, output = run('successful return still requires visual confirmation', case)
assert 'SIM_COMMAND recovery retry' in output and state['pending'] == 0

case = copy.deepcopy(BASE)
case['executions'][1]['board'] = AFTER_HUMAN
case['commands'][3:3] = [dict(phase='recovery', command='restored', board=AFTER_ROBOT),
                         dict(phase='recovery', command='retry')]
_, _, output = run('incorrect restore does not silently accept the move', case)
assert output.count('SIM_COMMAND recovery ') == 2

case = copy.deepcopy(BASE)
case['executions'][1]['success'] = 0
case['commands'].insert(3, dict(phase='recovery', command='accept'))
state, _, output = run('operator accepts already completed action after failed return', case)
assert 'SIM_COMMAND recovery accept' in output

case = copy.deepcopy(BASE)
case['executions'][1].update(success=0, board=AFTER_HUMAN)
case['executions'].insert(2, copy.deepcopy(BASE['executions'][1]))
case['commands'].insert(3, dict(phase='recovery', command='restored', board=AFTER_HUMAN))
run('explicit restore retains history and permits one replan', case, calls=4)

case = copy.deepcopy(BASE)
case['commands'].insert(2, dict(phase='human', command='ready', board=INITIAL))
_, _, output = run('unchanged player board does not consume a turn', case)
assert output.count('SIM_COMMAND human ready') == 3

case = copy.deepcopy(BASE)
case['queries'][1]['fail'] = 1
state, _, _ = run('engine failure is not an empty legal list', case, 'a3a4', 1, 1)
assert state['result'] == 0 and state['pending'] == 0 and state['phase'] == 'error'

case = copy.deepcopy(BASE)
case['queries'][1]['choice'] = 'b9b8'
state, _, _ = run('bestmove outside legal list issues no motion', case, 'a3a4', 1, 1)
assert state['pending'] == 0

case = copy.deepcopy(BASE)
case['queries'][0]['legal'] = []
state, _, _ = run('scripted terminal signal', case, '', calls=1)
assert state['result'] == 2 and state['phase'] == 'finished'

case = copy.deepcopy(BASE)
case['executions'][0]['success'] = 0
run('prepare failure does not start the game', case, '', 1, 1)

case = copy.deepcopy(BASE)
case['commands'][2] = dict(phase='human', command='resign')
state, _, _ = run('human resignation', case, '', calls=1)
assert state['result'] == 2

case = copy.deepcopy(BASE)
case['commands'] = case['commands'][:2] + [dict(phase='human', command='quit')]
case['queries'][0]['choice'] = 'a3a4'
case['queries'][1]['choice'] = ''
case['executions'] = [case['executions'][0], dict(purpose='move', move='a3a4', success=1, board=AFTER_HUMAN)]
run('robot can play red first', case, 'a3a4', calls=2, settings={'human_red': 0})

case = copy.deepcopy(BASE)
case['executions'][1].update(success=0, board=AFTER_HUMAN)
case['commands'].insert(3, dict(phase='recovery', command='quit'))
state, _, _ = run('quit preserves unconfirmed action', case, 'a3a4', calls=2)
assert state['pending'] == 1 and state['phase'] == 'stopped'

case = copy.deepcopy(BASE)
case['commands'] = [dict(phase='prepare', command='ready'),
                    dict(phase='confirm_start', command='ready', board=AFTER_HUMAN),
                    dict(phase='confirm_start', command='ready')]
case['queries'] = [dict(history='', legal=[])]
state, _, output = run('changed starting board is confirmed again', case, '', calls=1)
assert output.count('SIM_COMMAND confirm_start ready') == 2 and state['board'] == AFTER_HUMAN

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
