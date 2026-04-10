# YSC RTK + ZED 机器人导航项目说明

本项目用于机械狗在室外环境中的 RTK 定位、ZED 融合定位与多航点导航控制，支持：

- 通过 4G 模块串口读取 RTK 数据
- ZED 官方 GNSS/VIO geotracking 融合
- 航点采集与自动打点
- 多航点巡航、返航、暂停/恢复
- 与 Android/WebSocket 服务联动（任务下发、状态推送、停止任务）

---

## 1. 目录结构（核心文件）

- `rtk_cors_4g.py`：主程序，RTK+ZED 融合导航与状态机
- `robotdog_ws_client.py`：WebSocket 控制与 RTK 状态转发
- `auto/rtk_auto.py`：自动打点采集工具（直线段自动抽取航点）
- `config.json`：运行配置（Android 服务地址、端口、MySQL 等）
- `motion.py`：运动控制客户端（封装 M20 Pro）
- `nav_utils.py`：导航通用工具函数
- `route_segment_planner.py`：A/B 分段路线规划
- `ysc/`：统一业务包命名空间
- `ysc/db/`：路线持久化（MySQL）
- `ysc/zed/`：ZED 外参与相关封装
- `ysc/waypoint/`：航点数据结构
- `ysc/rtk_nav/`：`rtk_cors_4g.py` 抽离出的配置、任务控制、航点处理与导航数学模块

命名说明：

- 旧的 `unitree_*` 命名接口已移除，统一使用 `ysc/` 下的新模块入口。

## 2. 配置文件说明（`config.json`）

当前示例字段：

- `android_host`：Android 服务 IP
- `ws_port`：WebSocket 端口
- `api_port`：HTTP API 端口（如路线上传、stop-task）
- `status_http_port`：本地 RTK 状态 HTTP 服务端口
- `route_persistence_enabled`：是否启用路线持久化
- `mysql`：MySQL 连接信息

建议：

- 不要将生产密码明文提交到公共仓库
- 针对不同环境准备独立配置（开发/测试/生产）

---

## 3. 启动方式

导航主程序示例：

```bash
python rtk_cors_4g.py 1.json
```

指定机器人 IP：

```bash
python rtk_cors_4g.py 10.21.31.103 1.json
```

按上次任务进度恢复：

```bash
python rtk_cors_4g.py --resume-active
```

启动 WebSocket 控制与状态转发：

```bash
python robotdog_ws_client.py
```

说明：

- `python rtk_cors_4g.py 1.json` 仍兼容旧写法，会自动把第一个位置参数识别为航点文件
- `robotdog_ws_client.py` 本地只开放 `POST /robotdog/rtk`，用于接收导航状态后再通过 WebSocket 转发给前端

## 4. 自动打点流程（`auto/rtk_auto.py`）

用于走线采集路线，自动提取直线段起终点为航点。

启动示例：

```bash
python auto/rtk_auto.py 5.0 0.2 0.05
```

参数含义：

- `max_len`：单段最大长度（米）
- `max_dev`：允许的横向偏差（米）
- `min_step`：最小采样步进（米）

流程：

1. 启动后等待 RTK 原点建立
2. 回车开始自动采集
3. 再次回车结束采集
4. 输入路线名并可选上传到 Android 端
5. 保存航点到 `waypoints_*.json`

---

## 5. 航点文件格式

推荐新格式（对象数组）：

```json
[
  {
    "name": "1",
    "lon": 120.123456,
    "lat": 30.123456,
    "action_type": 0,
    "time": 0
  }
]
```

字段说明：

- `name`：点名
- `lon` / `lat`：经纬度
- `action_type`：
  - `0` 无动作
  - `1` 停留 `time` 秒
  - `2` 坐下 `time` 秒后起立
- `time`：动作持续时间（秒）

---

## 6. 运行逻辑概览

- 后台线程：
  - RTK 串口读取线程
  - ZED 融合定位线程
  - 地图绘制线程
- 任务控制：
  - `ysc/rtk_nav/mission.py`：恢复进度、target 解析、A/B 分段路线
  - `ysc/rtk_nav/waypoints.py`：航点文件加载、到点动作
  - `ysc/rtk_nav/navigation_math.py`：接近速度曲线、长距离中间点切分
- 主循环状态机：
  - `IDLE` 待机
  - `CRUISE` 巡航
  - `RETURN` 返航
  - `CHARGE` 预留
- 导航控制：
  - 远距离三阶段锁头（粗锁/细锁/吸附）
  - 近点联合速度调整（`vx/vy`）
  - 长距离自动插入中间点分段导航

---

## 7. 与外部服务联动

- 状态上报：`rtk_cors_4g.py` 向本机 `robotdog_ws_client.py` 的 `POST /robotdog/rtk` 推送 RTK 状态（默认 `8091`）
- 停止任务：调用 Android 端 `/api/stop-task`
- 路线上传：`auto/rtk_auto.py` 可调用 Android 端 `/api/robot-route/upload`
- 导航控制文件：
  - `tmp/robotdog_nav_pause_state.json`
  - `tmp/robotdog_nav_state.json`
