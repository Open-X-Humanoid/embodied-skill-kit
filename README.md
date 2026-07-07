# Embodied Skill Kit · TianYi 2.0 Secondary-Development Tutorial

**English** | [简体中文](README.zh-CN.md)

A set of graded, runnable examples with companion guides that takes secondary developers from *making a single body part move* all the way to *running a full task in a real scene*. Examples are distilled from three real deployed projects: box-moving, depalletizing, and toy-sorting.

## Five Stages (from atom to scene)

Content grows through five stages — the names keep a "build up from the smallest unit" metaphor; the subtitle is the plain-language version:

| Stage | In one line | Question it answers | Status |
|---|---|---|---|
| ① Atom | Make a single body part move | How do you move one part? Control interface, units, limits | ✅ In progress |
| ② Molecule | Coordinate parts into one motion | How do multiple parts coordinate? | 🚧 Planned |
| ③ Skill | Turn motion into a reliable task that reports success/failure | How does motion become reliable & report status? | 🚧 Planned |
| ④ Scene | Orchestrate skills into a full job | How are skills orchestrated into a job? | 🚧 Planned |
| ⑤ Evolution | Reshape the underlying capability (models / force control / IK) | How do you change the low-level capability? | 🚧 Planned |

Each stage directory splits into `demos/` (example code) and `docs/` (guides); **see the corresponding directory for the concrete list**.

## Quick Start

The fastest way in is to **start from the guides** — every stage's `docs/` folder holds self-contained walkthroughs you can read without the real robot; if you're new, begin with stage ① Atom (`atom/docs/`). The docs are the tutorial itself; the code is the subject they explain.

When you're ready to run on the real robot:

1. **Set up the environment & start body_control**: see *Prerequisite · Environment Setup* (`atom/docs/environment_setup.md`). Motion atoms need body_control running first; one-click start:

   ```bash
   ./scripts/start_body_control.sh        # run on the robot's x86 board, as user ubuntu
   ```

2. **Run an atom** (in a second terminal):

   ```bash
   source /home/ubuntu/ros2ws/install/setup.bash
   python3 atom/demos/atom01_head_ros2.py
   ```

3. Read the matching guide `atomNN_..._guide.md` to understand the code, then tweak-and-observe.

## Repository Layout

```
atom/
  demos/     example code (atomNN_<part>_<variant>.py; _robust = production version)
  docs/      guides — English `name.md` (default), Chinese `name_zh-CN.md`
  assets/    demo videos / rosbag recordings
scripts/     cross-stage utility scripts (e.g. one-click body_control startup)
```

## Notes

- Examples require the real robot (ROS2 + `bodyctrl_msgs`, etc.); they cannot run on a dev machine and are for static reading there.
- Docs are bilingual: the English version is `name.md` (default), the Chinese version is `name_zh-CN.md`.
- Most atoms ship a **plain version** and a **`_robust` production version** side by side: the plain one to learn the principle, the robust one adds state reading / limit checking / readiness waiting for production use.
