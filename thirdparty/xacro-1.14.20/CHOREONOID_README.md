# xacro 1.14.20 bundled for Choreonoid

This directory contains a subset of upstream xacro 1.14.20, bundled so that
Choreonoid can expand URDF xacro files without requiring a system ROS xacro
installation.

## Upstream

- Project: https://github.com/ros/xacro
- Version: tag `1.14.20`
- License: BSD 3-Clause, as declared in `package.xml` and upstream source
  headers.

## Included files

Only the files needed by Choreonoid's xacro execution path are included:

    README.md
    package.xml
    scripts/completion.bash
    scripts/cnoid-xacro
    src/xacro/*.py
    src/xacro/substitution_args.py
    CMakeLists.txt
    CHOREONOID_README.md

Upstream files for catkin packaging, tests, CI, setup.py, and CMake helper
macros are omitted because Choreonoid only copies the Python package into its
own `cnoid` Python namespace and installs the wrapper executable.

## Choreonoid changes

- `scripts/xacro` is installed as `scripts/cnoid-xacro`.
- The wrapper imports `cnoid.xacro` instead of `xacro` because Choreonoid
  installs the package below its `cnoid` Python namespace.
- `src/xacro/substitution_args.py` is bundled from the previous Choreonoid xacro
  subset and `src/xacro/__init__.py` imports it instead of ROS `roslaunch`.
  This keeps `$(find package_name)` usable in non-ROS Choreonoid deployments
  when `ROS_PACKAGE_PATH` is supplied by Choreonoid's xacro loader.
  Its `_eval_find` splits `ROS_PACKAGE_PATH` with `os.pathsep` instead of a
  hard-coded `':'` so the path list also works on Windows, where the separator
  is `';'` and entries contain drive letters such as `C:/...`.
- `CMakeLists.txt` is Choreonoid-specific and copies/installs the subset above.

When updating this directory, start from the upstream release tag, copy the
same subset of files, reapply the wrapper import change and local substitution
argument resolver import, and update the version numbers in this file and the
top-level `CMakeLists.txt`.
