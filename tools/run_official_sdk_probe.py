"""Run each read-only SDK API in a separate process with a hard time limit."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--exe', type=Path, required=True)
parser.add_argument('--ip', default='169.254.138.6')
parser.add_argument('--port', type=int, default=8899)
parser.add_argument('--process-timeout', type=float, default=15)
parser.add_argument('--output', type=Path, default=Path('downloads/official-sdk-probe-results.json'))
args = parser.parse_args()
exe = args.exe.resolve()
if exe.name != 'official_sdk_probe.exe':
    parser.error('--exe must refer to the read-only official_sdk_probe.exe')

def decode(data):
    try:
        return (data or b'').decode('utf-8')
    except UnicodeDecodeError:
        return (data or b'').decode('gb18030', errors='replace')

dll = exe.parent / 'aubo_sdk.dll'
report = {
    'ip': args.ip, 'port': args.port, 'exe': str(exe), 'dll': str(dll),
    'dll_sha256': hashlib.sha256(dll.read_bytes()).hexdigest(), 'tests': []
}
for api in ('rpc', 'rs', 'service'):
    command = [str(exe), api, '--ip', args.ip, '--port', str(args.port), '--timeout-ms', '3000']
    started = time.monotonic()
    print(f'Testing {api} at {args.ip}:{args.port} ...', flush=True)
    try:
        result = subprocess.run(command, capture_output=True, timeout=args.process_timeout)
        item = {'api': api, 'exit_code': result.returncode, 'timed_out': False,
                'stdout': decode(result.stdout), 'stderr': decode(result.stderr)}
    except subprocess.TimeoutExpired as error:
        item = {'api': api, 'exit_code': None, 'timed_out': True,
                'stdout': decode(error.stdout), 'stderr': decode(error.stderr)}
    item['elapsed_seconds'] = round(time.monotonic() - started, 3)
    item['read_completed'] = not item['timed_out'] and item['exit_code'] == 0 and \
        'stage=read_completed result=0' in item['stdout']
    report['tests'].append(item)
    print(json.dumps(item, ensure_ascii=False, indent=2), flush=True)
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print(f'Results saved to {args.output}', flush=True)
