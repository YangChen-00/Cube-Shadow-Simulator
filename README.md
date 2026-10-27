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

`pick` 会优先在完整碰撞约束下求解；如果低位抓取因 Franka 保守碰撞球触碰桌面安全边界而无解，仅对该抓取接触段临时允许供应桌面接触，并由 cuRobo 再规划一次。该许可持续到 `lift` 起步，方块离开桌面后立即恢复桌面碰撞。在 `place` 的最后一小段，仅临时放宽方块与 JSON 中声明的目标支撑面，以及经旋转足迹确认不会发生实体重叠、但落入保守 attached-sphere 外扩范围的同层相邻方块；这是为了避免四球方块代理大于真实 40 mm 方块所产生的误报。刚放下的方块会在夹爪完成 `retreat` 后重新成为碰撞障碍。其他桌面、方块、墙面以及机械臂自碰撞仍由 cuRobo 检查。任意一段没有解，整个布局即判为不可执行，并记录失败方块、阶段和 cuRobo 状态。

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
“录制并下载完整装配视频”。Simulator 会按当前浏览器视角逐帧渲染 cuRobo
轨迹，导出固定 30 FPS 的 H.264 MP4，然后由浏览器自动下载。选择的速度
会改变成片时长，不会重新运行运动规划。同一份 MP4 也会保存到 `--output`
指定的目录。录制时请保持 Viser 页面连接，并在开始前调整好镜头。

## 双机械臂 U → S → T 演示

`sequence` 会加载 Studio 的 U、S、T 三份布局，把三个字母沿桌面 X 方向错开放置，并在场景两侧建立两台由同一机器人 YAML 派生的机械臂。左臂装 U，右臂依次装 S、T；整个流程严格串行，避免未经验证的双臂同步运动。两个基座和各自供料点沿桌面 X 方向一起平移，供料点不会落入 S 的建造区域。每个字母结束后，cuRobo 会规划回待机位；若回位无解，后续同一机械臂会直接从上一段真实终点继续规划，不会在画面中跳回默认姿态。已经完成的字母会保留在墙前，并作为静态方块障碍加入后续字母的 cuRobo 场景。

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

默认机械臂基座间距为 `0.56 m`，字母中心间距为 `130 mm`；可用 `--arm-separation` 和 `--letter-spacing` 覆盖。两台机械臂使用各自变换后的标定坐标和供料点，但共用机器人 YAML。正式录屏前应根据远程场景实测调整基座间距。当前方案只允许一台机械臂运动；cuRobo 对活动机械臂做自碰撞、环境、携带方块和已完成字母碰撞检查，另一台机械臂依靠分区和串行策略避让，不宣称做了动态双臂互碰规划。

网页中的“播放 U → S → T 双臂装配”会连续播放全部轨迹；“录制并下载 U-S-T 完整视频”会按所选速度和清晰度导出整段 MP4。

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
- `runs/UST/sequence_manifest.json`：双臂分工、串行顺序、间距、回位状态和三次规划结果；
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
