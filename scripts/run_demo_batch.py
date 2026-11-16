"""Run the five README examples with the unmodified planner objective."""
import argparse
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
CASES = [('nav', 5), ('nav', 6), ('nav', 8), ('man', 5), ('man', 6)]

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenarios', nargs='+', type=int, default=list(range(1, 6)))
    args = parser.parse_args()
    destination = ROOT / 'data/presentation'
    destination.mkdir(parents=True, exist_ok=True)
    for scenario in args.scenarios:
        task, case = CASES[scenario-1]
        output = destination / f'scenario_{scenario:02d}'
        pending = output.with_suffix('.pending.json')
        pending.unlink(missing_ok=True)
        command = [sys.executable, 'main.py', f'--task={task}', f'--case={case}', '--seed=42',
                   '--plan-only', f'--export-replay={pending}']
        print('Running', scenario, flush=True)
        with output.with_suffix('.log').open('w') as log:
            subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True,
                           env={**os.environ, 'PYTHONHASHSEED': '0', 'MPLBACKEND': 'Agg'})
        if not pending.exists():
            raise RuntimeError(f'No solved replay for scenario {scenario}; see {output}.log')
        pending.replace(output.with_suffix('.json'))
        print('Saved', output.with_suffix('.json'), flush=True)
