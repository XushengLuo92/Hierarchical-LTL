"""Package the website without the machine-specific ROS workspace symlink."""
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]

if __name__ == '__main__':
    destination = Path(sys.argv[1])
    destination.mkdir(parents=True, exist_ok=False)
    for name in ('index.html', '.nojekyll', 'Hierarchical_LTL.pdf'):
        shutil.copy2(ROOT/name, destination/name)
    for name in ('site', 'imgs', 'demo'):
        shutil.copytree(ROOT/name, destination/name)
    print(f'Website packaged in {destination}')
