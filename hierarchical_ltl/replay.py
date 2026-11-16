"""Export solved plans without modifying assignment, timing, or motion."""
import json
from pathlib import Path


def export_replay(args, workspace, specification, network, run, identities, paths):
    data = {
        'schema_version': 1, 'task': args.task, 'case': args.case, 'seed': args.seed,
        'execution': 'original GMAPP paths' if paths is not None else 'high-level assembly schedule',
        'hierarchy': specification,
        'identities': [{'identifier': key, 'robots': value}
                       for key, value in identities.items()],
        'stages': run,
        'precedence': [[list(a), list(b)] for a, b in network.edges],
        'workspace': {'width': workspace.width, 'height': workspace.length,
                      'regions': workspace.regions, 'obstacles': workspace.obstacles},
        'robots': [{'type': robot[0], 'index': robot[1], 'initial': point,
                    'path': paths.get(robot, [point]) if paths is not None else []}
                   for robot, point in workspace.type_robot_location.items()],
    }
    destination = Path(args.export_replay)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(data, indent=2) + '\n')
    print(f'Replay saved: {destination}')
