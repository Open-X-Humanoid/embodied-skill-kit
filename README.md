# Embodied Skill Kit · TianYi 2.0 Secondary-Development Tutorial

**English** | [简体中文](README.zh-CN.md)

A set of graded, runnable examples with companion guides that takes secondary developers from *making a single body part move* all the way to *running a full task in a real scene*. Examples are distilled from three real deployed projects: box-moving, depalletizing, and toy-sorting.

## Three Stages (from atom to scene)

Content grows through three stages — from isolated control primitives, to reusable robot capabilities, and finally to complete jobs:

| Stage | In one line | Question it answers | Status |
|---|---|---|---|
| ①&nbsp;Atom | Make a single body part move | How do you move one part? Control interface, units, limits | ✅ In progress |
| ②&nbsp;Skill | Coordinate perception and motion into a reusable capability | How does a robot complete one meaningful task? | ✅ Finger Tap and Bottle Grasp available |
| ③&nbsp;Scene | Orchestrate skills into a full job | How are skills orchestrated into a job? | 🚧 Planned |

## Quick Start

The fastest way in is to **start from the guides** — every stage's `docs/` folder holds self-contained walkthroughs you can read without the real robot; if you're new, begin with stage ① Atom — each module has its own README (`atom/motion/`, `atom/perception/`, `atom/interaction/`). The docs are the tutorial itself; the code is the subject they explain.

When you're ready to run on the real robot:

1. **Set up the environment & start body_control**: see *Prerequisite · Environment Setup* (`docs/environment_setup.md`). Motion atoms need body_control running first; one-click start:

   ```bash
   ./scripts/start_body_control.sh        # run on the robot's x86 board, as user ubuntu
   ```

2. **Run an atom** (in a second terminal):

   ```bash
   source /home/ubuntu/ros2ws/install/setup.bash
   python3 atom/motion/motion01_head_ros2.py
   ```

3. Read the matching guide `atomNN_..._guide.md` to understand the code, then tweak-and-observe.

Continue with [Skill 1 · Finger Tap](skill/skill01_finger_tap/docs/skill01_finger_tap_guide.md) when you are ready to combine perception and arm control.

Then explore [Skill 2 · Bottle Grasp](skill/skill02_bottle_grasp/docs/skill02_bottle_grasp_guide.md) to combine object perception, box geometry, arm motion, and dexterous-hand control.

## Repository Layout

```
docs/         prerequisites shared by every stage (environment setup, frames, conventions)
atom/
  motion/      joint & chassis control  — code at root, docs/, assets/
  perception/  camera, force, power, TF
  interaction/ voice in & out
               (each module: README.md + code at root + docs/ + assets/)
skill/
  skill01_finger_tap/   AprilTag perception + QP arm control for one complete tap
  skill02_bottle_grasp/ bottle/box perception + MoveIt/QP control for grasp and placement
scene/                  multi-skill task orchestration (planned)
scripts/     cross-stage utility scripts (e.g. one-click body_control startup)
```

## Notes

- Examples require the real robot (ROS2 + `bodyctrl_msgs`, etc.); they cannot run on a dev machine and are for static reading there.
- Docs are bilingual: the English version is `name.md` (default), the Chinese version is `name_zh-CN.md`.
- Most atoms ship a **plain version** and a **`_robust` production version** side by side: the plain one to learn the principle, the robust one adds state reading / limit checking / readiness waiting for production use.
