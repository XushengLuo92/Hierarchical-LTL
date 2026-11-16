# Project page

Buildless GitHub Pages site. Edit `demos.js` to update the four task descriptions and their paper sources. `app.js` provides accessible scenario switching and `?demo=N` links.

The Pages workflow packages only website assets with `scripts/package_site.py`, then deploys them. This avoids the original ROS workspace's machine-specific `CMakeLists.txt` symlink. Pushes to `master` that change website assets trigger deployment automatically.

The page deliberately omits planner metadata, download-link rows, and reproduction sections. Installation and execution instructions belong in the repository README.

The house is displayed as Scenario 4 and retains demo ID 5 so existing house links continue to work. Its player uses the original `arm_simulation.mp4` recording and poster.
