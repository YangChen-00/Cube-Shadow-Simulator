# Cube Shadow Simulator（cuRoboV2 + Viser）

本项目是 `Cube-Shadow-Studio` 的机器人端。Studio 把手绘阴影转换成有顺序和支撑关系的方块布局；Simulator 在远程 NVIDIA GPU 服务器上用 **cuRoboV2** 做 IK、抓取规划、轨迹优化、自碰撞、环境碰撞和携带物碰撞检查，再用 **cuRobo 自带的 ViserVisualizer** 播放真实关节轨迹、方块搬运过程以及固定点光源产生的墙面阴影。

项目不再包含自制运动学、PyBullet 后端或 HTML 假动画。保存和播放的所有机械臂轨迹只来自 `MotionPlanner.plan_grasp()`、`MotionPlanner.plan_pose()` 和返回待机位的 `MotionPlanner.plan_cspace()`；不会在规划失败处补假动画。

## 流程

```text
手绘 PNG
  -> Cube-Shadow-Studio layout.json（mm、放置顺序、支撑关系）
  -> 静态承托 / 重力 / 方块重叠检查
  -> 标定变换 tabletop_in_robot_base
  -> cuRobo Scene（桌面、投影墙、供应区方块、已放置方块）
  -> 每块：plan_grasp(pre_pick + pick) -> plan_pose(lift + pre_place + place + retreat)
  -> 每个字母完成后：plan_cspace(return home)
  -> 携带方块作为 4 个 attached collision spheres
  -> 每放一块，更新 cuRobo collision world
  -> Viser 播放关节轨迹与动态阴影
```

`pick` 会优先在完整碰撞约束下求解；如果低位抓取因 Franka 保守碰撞球触碰桌面安全边界而无解，仅对该抓取接触段临时允许供应桌面接触，并由 cuRobo 再规划一次。该许可持续到 `lift` 起步，方块离开桌面后立即恢复桌面碰撞。在 `place` 的最后一小段，临时放宽目标支撑面，以及落在四球代理外扩范围内的相邻方块（含正下方一层）。若这次下降仍然无解，会再放宽一立方块宽度内的同层邻居和桌面，并由 cuRobo 重规划一次。支撑面和刚放下的方块会保持放开，直到夹爪完成 `retreat`，避免手指还停在抓取位时起点就被判碰撞。其他墙面以及机械臂自碰撞仍由 cuRobo 检查。任意一段没有解，整个布局即判为不可执行，并记录失败方块、阶段和 cuRobo 状态。

## 远程服务器安装

要求 Python 3.10+、NVIDIA GPU、匹配的 CUDA/PyTorch，以及 `nvidia-curobo >= 0.8.0`。cuRoboV2 是 0.8 起的新 API，不能使用旧版 `MotionGen` 代码。

先按 NVlabs 官方说明在远程服务器安装 cuRoboV2，然后安装本项目：

```bash
git clone https://github.com/NVlabs/curobo.git
cd curobo
pip install -e ".[cu12-torch]"

cd /path/to/robo_cube/Cube-Shadow-Simulator
pip install -e .
```

如果服务器已经有课程提供的 cuRobo 环境，只执行最后一条即可。

## 必须先做的标定

复制并修改 [`config/franka_remote.json`](config/franka_remote.json)：

- `robot_yaml`：课程服务器上的 cuRoboV2 机器人配置；
- `tabletop_in_robot_base`：Studio 桌面坐标系到机器人 base 的实测位姿；
- `tool_down_quaternion_tabletop_wxyz`：向下抓取时 TCP 姿态；
- `tcp_to_cube_center_m`：抓住方块后 TCP 到方块中心的竖直距离；
- `initial_joint_positions`：可填实测初始关节角；为 `null` 时使用机器人 YAML 的默认关节位姿；
- `cube_source`：方块供应区。默认 `feeder` 模式在标定取料点逐块激活方块，避免大布局中远端托盘格不可达；`grid` 模式可用于实体托盘。

仓库给出的数值只是 Franka 桌面演示模板，不是实机标定结果。最终录屏前必须换成课程服务器的机器人 YAML 和标定值。

## 运行完整演示

在远程服务器运行现有 Studio 样例 T：

```bash
cd /path/to/robo_cube/Cube-Shadow-Simulator
cube-shadow-sim demo \
  ../Cube-Shadow-Studio/handoff/current_examples/T.json \
  --config config/franka_remote.json \
  --target-image ../Cube-Shadow-Studio/web/samples/T.png \
  --output runs/T
```

Viser 默认仅监听服务器 `127.0.0.1:8080`。在本机建立安全转发：

```bash
ssh -L 8080:127.0.0.1:8080 USER@SERVER
```

然后打开 `http://127.0.0.1:8080`，点击“播放完整装配”。

网页中可直接选择视频清晰度和 `0.25×–4×` 下载速度，再点击
“录制并下载完整装配视频”。Simulator 会按当前浏览器视角渲染 cuRobo
轨迹，导出 30 FPS 的 H.264 MP4，然后由浏览器自动下载。为了缩短等待，
每秒成片只向浏览器抓取 10 帧，再重复写成 30 FPS。选择的速度只改变成片
时长，不会重新运行运动规划。同一份 MP4 也会保存到 `--output`
指定的目录。录制时请保持 Viser 页面连接，并在开始前调整好镜头。

## 双机械臂 U + T → S 演示

`sequence` 会加载 Studio 的 U、S、T 三份布局，把三个字母沿桌面 X 方向错开放置。两台机械臂站在字母左右两侧、面向工件：左臂在 `-X`，右臂在 `+X`。左臂装配 U，右臂同时装配 T；两侧都结束后，右臂再装配中间的 S。同一阶段里的两台臂按各自轨迹同时播放。cuRobo 仍逐臂规划，不把另一台臂当作动态障碍，因此左右基座需要分开，避免轨迹在中间交叉。

每个字母结束后，cuRobo 会规划回待机位。回位成功时，同一台臂的下一个字母从该终点继续，画面不会跳变。若某个字母规划失败，下一字母改从待机位重新规划，避免停在失败姿态里导致后面的字母也无法开始。已经放下的方块会留在墙前，并作为静态障碍加入后续规划。供料点在各自基座朝向字母的一侧、并偏到 S 的前方，避免和字母重叠。

```bash
cube-shadow-sim sequence \
  ../Cube-Shadow-Studio/handoff/current_examples/U.json \
  ../Cube-Shadow-Studio/handoff/current_examples/S.json \
  ../Cube-Shadow-Studio/handoff/current_examples/T.json \
  --target-images \
    ../Cube-Shadow-Studio/web/samples/U.png \
    ../Cube-Shadow-Studio/web/samples/S.png \
    ../Cube-Shadow-Studio/web/samples/T.png \
  --config config/franka_remote.json \
  --output runs/UST
```

默认左右基座间距为 `0.96 m`，基座在桌面坐标系的 Y 为 `-0.20 m`，字母中心间距为 `240 mm`。可用 `--arm-separation`、`--arm-y` 和 `--letter-spacing` 覆盖。两台机械臂共用机器人 YAML，工具朝下的标定保持不变，右臂基座绕竖直轴转 180° 面向左侧。正式录屏前应根据场景调整基座间距。cuRobo 对每台活动机械臂做自碰撞、环境、携带方块和已完成字母碰撞检查。

网页中的“播放左右协同装配”会同时播放 U 和 T，再播放 S；“录制并下载 U-S-T 完整视频”会按所选速度和清晰度导出整段 MP4。

若远程环境仍报错，可在原命令末尾增加 `--debug` 输出完整 traceback。Franka
配置会在规划时使用 7 个主动臂关节，但插值轨迹可能包含 2 个被锁定的夹爪关节；
Simulator 会按关节名把 9 维轨迹端点投影回 7 维规划空间，并保留完整轨迹供 Viser
播放。

只规划并保存证据、不启动网页：

```bash
cube-shadow-sim plan \
  ../Cube-Shadow-Studio/handoff/current_examples/T.json \
  --config config/franka_remote.json \
  --output runs/T
```

从新的手绘图一键联动 Studio：

```bash
cube-shadow-sim demo \
  --drawing ../Cube-Shadow-Studio/web/samples/T.png \
  --studio-root ../Cube-Shadow-Studio \
  --budget 32 --seconds 5 \
  --config config/franka_remote.json \
  --output runs/from-drawing
```

## 输出证据

- `manifest.json`：输入布局 SHA-256、Studio 阴影指标、标定配置和技术栈；
- `structural_feasibility.json`：支撑、重力、顺序、边界与静态重叠检查；
- `curobo_plan.json`：每一段 cuRobo 成功/失败状态和轨迹点数；
- `curobo_trajectories.npz`：Viser 播放的真实关节轨迹与抓取相对变换。
- `cube-shadow-<speed>x-<timestamp>.mp4`：网页导出的 30 FPS 完整装配视频。
- `runs/UST/sequence_manifest.json`：双臂分工、同时装配的阶段、间距、回位状态和三次规划结果；
- `runs/UST/handoff/{U,S,T}.json`：为组合场景平移后的 Studio 布局副本；
- `cube-shadow-U-S-T-<speed>x-<timestamp>.mp4`：双臂序列网页导出的完整视频。

故意失败的结构样例：

```bash
cube-shadow-sim plan examples/infeasible_floating.json \
  --config config/franka_remote.json \
  --output runs/infeasible
```

它会在移动机械臂前以 `FLOATING_CUBE` 拒绝。机器人不可达、环境碰撞或无碰撞轨迹不存在时，则由 cuRobo 在对应 phase 返回失败。

## 评分点对应

| 评分要求 | 实现证据 |
|---|---|
| 基本机械臂和 pick-and-place | cuRoboV2 机器人模型与六阶段抓放轨迹 |
| 基本支撑约束 | Studio 支撑顺序 + Simulator 静态承托 gate |
| 输入到装配完整流程 | `--drawing` 直接调用 Studio 后继续 cuRobo 规划与 Viser 展示 |
| 碰撞检测 | cuRobo 自碰撞、Scene collision、attached cube collision spheres |
| 重力约束 | 每个放置前缀的支撑多边形与组合质心检查；浮空直接拒绝 |
| 可执行性评估 | 每个方块每个阶段必须由 cuRobo 返回成功 |
| 16×16 目标 | 接口按方块数规划，不绑定图像分辨率；可读取 Studio 的 16×16/320×320 掩膜结果 |
| 成功和失败两类图案 | Studio T/S/U 成功候选；`infeasible_floating.json` 和 cuRobo phase failure 报告 |

## 安全边界

Viser 是规划结果展示，不直接下发真实机械臂。若后续连接硬件控制器，必须额外加入急停、速度限制、夹爪 I/O、状态反馈、轨迹时间同步和现场人员监护；不能把可视化成功当作实机安全认证。
