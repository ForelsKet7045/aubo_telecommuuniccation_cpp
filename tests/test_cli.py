"""Offline integration tests against the socket-free mock_sdk.dll."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys

exe = Path(sys.argv[1]).resolve()
assert exe.name == 'robot_communication_mock.exe', 'Only run these tests against the offline executable'
checks = 0

def run(args, scenario='', code=0):
    global checks
    env = {**os.environ, 'AUBO_TEST_SCENARIO': scenario}
    result = subprocess.run([str(exe), *args], env=env, capture_output=True, text=True, timeout=5)
    assert result.returncode == code, (args, scenario, result.returncode, result.stderr)
    events = [json.loads(line[5:]) for line in result.stderr.splitlines() if line.startswith('MOCK ')]
    checks += 1
    return result, events

def called(events, name):
    return any(event['call'] == name for event in events)

run(['--help'])
for args in [
    ['--port', '0'], ['--port', '65536'], ['--port', '1.5'], ['--ip'],
    ['read', '--hz', '10'], ['watch', '--hz', '0'], ['watch', '--seconds', '0'],
    ['move'], ['move', '--delta-mm', '1', '2'],
    ['move', '--pose', 'nan', '0', '0', '0', '0', '0'], ['--user', 'aubo'],
    ['read', '--dry-run'],
]:
    result, events = run(args, code=1)
    assert not called(events, 'login'), 'Invalid arguments must fail before login'

result, events = run(['read', '--ip', 'offline'])
sample = json.loads(result.stdout)
assert sample['pose_reference'] == 'flange' and sample['frame'] == 'base'
assert sample['xyz_mm'] == [100, 200, 300]
assert sample['quaternion_wxyz'] == [1, 0, 0, 0]
for radians, degrees in zip(sample['joints_rad'], sample['joints_deg']):
    assert math.isclose(math.degrees(radians), degrees)
assert any(event.get('port') == 8899 for event in events)
assert not called(events, 'move_line')
assert all(called(events, name) for name in ['logout', 'destroy', 'uninitialize'])

result, _ = run(['watch', '--seconds', '0.25', '--hz', '10'])
assert len([json.loads(line) for line in result.stdout.splitlines()]) >= 2
result, events = run(['watch', '--seconds', '1'], 'read-failure', code=1)
assert called(events, 'logout') and not called(events, 'move_line')
_, events = run(['read'], 'login-failure', code=1)
assert called(events, 'destroy') and called(events, 'uninitialize')
assert not called(events, 'logout')
run(['read'], 'disconnected', code=1)

delta = ['move', '--delta-mm', '10', '-20', '30']
faults = {
    'power-off': 'armPowerStatus=0', 'emergency': 'remoteEmergency=1',
    'soft-emergency': 'softEmergency=1', 'collision': 'robotCollision=1',
    'force-mode': 'forceControlMode=1', 'can-error': 'armCanbusStatus=4',
    'encoder-error': 'encoderErrorStatus=1', 'encoder-lines-error': 'encoderLinesError=1',
    'joint-error': 'jointErrorStatus=1', 'joint-collision': 'jointCollisionDetect=2',
    'singularity': 'singularityOverSpeedAlarm=1', 'current-alarm': 'robotCurrentAlarm=1',
}
for scenario, reason in faults.items():
    result, events = run(delta, scenario, code=1)
    assert reason in result.stderr, (scenario, result.stderr)
    assert not called(events, 'move_line') and not called(events, 'configure'), scenario

for scenario in ['preexisting', 'ik-failure']:
    _, events = run(delta, scenario, code=1)
    assert not called(events, 'move_line'), scenario

result, events = run(['diagnose'])
diagnosis = json.loads(result.stdout)
assert diagnosis['diagnosis']['remoteHalt_raw'] == 1
assert diagnosis['diagnosis']['remoteEmergency'] == 0
assert diagnosis['motion_blockers'] == []
assert not called(events, 'move_line') and not called(events, 'configure')
result, events = run(['diagnose'], 'emergency')
assert json.loads(result.stdout)['motion_blockers'] == ['remoteEmergency=1']

result, events = run(delta + ['--dry-run'])
preview = json.loads(result.stdout)
assert preview['dry_run'] is True
assert all(math.isclose(a, b) for a, b in zip(preview['target_position_m'], [.11, .18, .33]))
assert not any(called(events, name) for name in ['configure', 'move_line', 'stop'])
assert called(events, 'logout')

result, events = run(delta)
position = next(event['position'] for event in events if event['call'] == 'ik')
assert all(math.isclose(a, b) for a, b in zip(position, [0.11, 0.18, 0.33]))
assert called(events, 'move_line') and not called(events, 'stop')
assert 'Arrived' in result.stderr
result, events = run(['move', '--pose', '.2', '.3', '.4', '0', '0', '0'])
assert next(event['position'] for event in events if event['call'] == 'ik') == [.2, .3, .4]

for scenario in ['timeout', 'paused', 'line-failure']:
    _, events = run(delta + ['--timeout', '0.25'], scenario, code=1)
    assert called(events, 'move_line') and called(events, 'stop'), scenario
    assert called(events, 'logout'), scenario

print(f'PASS: {checks} offline CLI/controller scenarios; no device connections')
