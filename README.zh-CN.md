# Embodied Skill Kit · 天轶 2.0 二次开发教学库

[English](README.md) | **简体中文**

一套「能跑通、看得见效果」的分级示例 + 中文讲解文档，带二次开发者从**让单个部位动起来**，一步步走到**在真实场景里跑通整套作业**。示例取自搬箱 / 拆垛 / 分拣三个真实落地项目。

## 五个阶段（从原子到场景）

内容按五个阶段生长——名字沿用「由小到大搭建」的隐喻，副标题是大白话：

| 阶段 | 一句话 | 回答的问题 | 状态 |
|---|---|---|---|
| ①&nbsp;原子 Atom | 让单个部位动起来 | 怎么让一个部位动？控制接口、单位、限位 | ✅ 进行中 |
| ②&nbsp;分子 Molecule | 多部位协调成一个动作 | 多个部位怎么协调？ | 🚧 规划中 |
| ③&nbsp;技能 Skill | 动作变成能报成败的可靠任务 | 动作怎么变可靠、报成败？ | 🚧 规划中 |
| ④&nbsp;场景 Scene | 技能编排成整套作业 | 技能怎么编排成作业？ | 🚧 规划中 |
| ⑤&nbsp;进化 Evolution | 改造底层能力（模型/力控/IK） | 怎么改底层能力？ | 🚧 规划中 |

每个阶段目录内分 `demos/`（示例代码）与 `docs/`（讲解文档），**具体清单见对应目录**。

## 快速开始

最快的入门方式是**从讲解文档读起**——每个阶段目录下的 `docs/` 都是独立成篇的 guide，不接真机也能看懂；新手从 ① 原子 `atom/docs/` 起步。**文档是教程主线，代码是讲解的对象。**

要在真机上跑，按三步：

1. **配环境 + 启动 body_control**：见《前置 · 环境配置》(`atom/docs/environment_setup_zh-CN.md`)。运动类原子需先起 body_control，可一键启动：

   ```bash
   ./scripts/start_body_control.sh        # 在机器人 x86 上、ubuntu 用户执行
   ```

2. **跑一个原子**（另开终端）：

   ```bash
   source /home/ubuntu/ros2ws/install/setup.bash
   python3 atom/demos/atom01_head_ros2.py
   ```

3. 对照同名文档《atomNN_..._guide.md》理解代码、改一改看变化。

## 仓库结构

```
atom/
  demos/     示例代码（atomNN_<部位>_<变体>.py，_robust 为生产版）
  docs/      讲解文档 — 英文 `name.md`（默认），中文 `name_zh-CN.md`
  assets/    演示视频 / rosbag 录制
scripts/     跨阶段工具脚本（如一键启动 body_control）
```

## 说明

- 示例依赖真机（ROS2 + `bodyctrl_msgs` 等），开发机无法运行，只做静态阅读。
- 文档中英双语：英文版为 `name.md`（默认），中文版为 `name_zh-CN.md`。
- 多数原子有**简洁版**与 **`_robust` 生产版**并存：简洁版看懂原理，生产版是加了状态读取 / 限位校验 / 就绪等待的可上业务写法。
