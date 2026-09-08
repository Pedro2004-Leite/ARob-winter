# Reference book - Small Unmanned Aircraft: Theory and Practice

This folder holds the companion material for the course's main textbook:

Randy Beard, Tim McLain, *Small Unmanned Aircraft: Theory and Practice*, Princeton University Press, 2012.

The material here was pulled from the authors' own public repository, [byu-magicc/mavsim_public](https://github.com/byu-magicc/mavsim_public), which is licensed under the GPLv3 (see [`LICENSE`](LICENSE)).
The full, original upstream README - with links to the chapter-by-chapter slide decks (PDF and PowerPoint), video solutions, and further supplemental material - is preserved at [`UPSTREAM_README.md`](UPSTREAM_README.md).

## The book PDF itself

The upstream repository does not version the book PDF - it only links to a Google Drive copy (see [`UPSTREAM_README.md`](UPSTREAM_README.md)).
[`uavbook.pdf`](uavbook.pdf) (2nd edition, 401 pages) is included here directly, next to its companion code.

## What is actually in this folder

The upstream repo does not contain the book text - it contains the simulation project that the book's end-of-chapter exercises build up, in four parallel implementations:

| Folder | Language / tool | Organized by | Notes |
|---|---|---|---|
| [`mavsim_simulink/`](mavsim_simulink/) | Simulink | book chapter (`chap2` … `chap12`) | Closest match to how this course's own labs are built (Simulink-based). |
| [`mavsim_matlab/`](mavsim_matlab/) | MATLAB scripts | book chapter (`chap2` … `chap12`) | Same structure as `mavsim_simulink`, without the Simulink block diagrams. |
| [`mavsim_python/`](mavsim_python/) | Python | software module (`models/`, `controllers/`, `estimators/`, `planners/`, `viewers/`, ...) | The actively maintained version; setup instructions in its own `README.md`. |
| [`legacy_mavsim_python/`](legacy_mavsim_python/) | Python | book chapter (`chap2` … `chap12`) | Older Python version, chapter-organized like the MATLAB/Simulink ones - useful if you want the Python API but still want to navigate by chapter number. |

All four simulate the same aircraft: the fixed-wing **Aerosonde** MAV used throughout the book's exercises (see `parameters/aerosonde_parameters.py` or equivalent in each folder).
This is **not** the AR.Drone quadrotor used in this course's own labs (see `../labs-25-26/`) - it is a fixed-wing vehicle, so the airframe, actuators, and aerodynamic model differ.
What carries over directly is the underlying theory: rigid-body dynamics, sensor models, state estimation, and the control/guidance structure, which is exactly what the course reuses from the book (see the chapter mapping below).

## How this maps to the course's own chapters

This course's chapter numbers (0-9) do **not** match the book's chapter numbers.
Per [`../lectures-22-23/COURSE_NOTES.md`](../lectures-22-23/COURSE_NOTES.md), the confirmed mapping is:

| Course chapter | Book chapter | Topic | Code folder to look at |
|---|---|---|---|
| Ch5 | Ch7 | Sensors | `chap7/` |
| Ch6 | Ch8 | State estimation | `chap8/` |
| Ch8 | Ch10 | Waypoint and orbit following | `chap10/` |
| Ch9 | Ch11 | Path management | `chap11/` |

For anything not in that table (rigid-body dynamics, forces and moments, linear/nonlinear control design), match by topic against the chapter titles listed in [`UPSTREAM_README.md`](UPSTREAM_README.md) rather than by chapter number.
