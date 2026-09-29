# control_center_chassis_guard

控制中心手动驾驶（导航 WASD / 建图 WASD）的底盘所有权租约、输入去抖与速度平滑。

## 为什么存在

1. **所有权租约**：厂商机械臂控制器在未被声明让位时，会以 ~62.5 Hz 向共享
   底盘目标话题重发零 `Twist`。厂商让位契约是 Redis DB0 键
   `shared_params:{domain}:{robot}:is_navigating`（裸字符串 `true`/`false`）。
2. **输入去抖**：2026-09-10 独立验收发现，持续按住 WASD 时控制中心上游
   `/manual_cmd_vel` 自身就含约 19%～21% 的单帧零速度。X11（含 NoMachine 远控）
   会把按住的键以“真 release + 立即 press”（`isAutoRepeat()==False`）成对投递，
   按键集合瞬时清空 → UI 自己发出单帧零速度 → 底盘顿挫，并使租约反复释放/获取。
3. **速度平滑**：2026-09-15。旧实现把键盘请求的速度**原样**下发，按 W 是
   0 → 0.20 m/s 的阶跃，松键是 0.20 → 0 的阶跃。本机上半身是人形、重心偏高，
   阶跃会让起步晃、停车时上身惯性摆动。

## 用法

```python
from control_center_chassis_guard import (
    ChassisControlLease, ManualKeyFilter, VelocityRamp, VelocityRampDiagnostics,
)

lease = ChassisControlLease(
    topic_suffix='0_306',            # 或 topic_hint='/topic_gv_front_lidar_0_306'
    logger=node.get_logger(),
    on_lease_lost=lambda reason: ...,
)
if lease.acquire():                  # 首个非零速度前必须成功；失败则只发零
    publish_nonzero(...)
publish_zero(...)
lease.release()                      # 归零并立即归还

keys = ManualKeyFilter(release_delay_sec=0.03)
keys.press(key)                      # 立即生效
keys.release(key)                    # 延迟 30ms 生效；同键 press 到来自动取消
keys.expire()                        # 由单发定时器驱动；返回集合是否变化
keys.clear()                         # 安全路径：X 键/失焦/切页/关闭，立即清空

ramp = VelocityRamp(diagnostics=VelocityRampDiagnostics(tag='nav'))
ramp.set_target(0.20, 0.0, 0.0)      # 键盘只改目标，立即生效、不延迟
each_control_tick():
    publish(ramp.step())             # 实际下发值按加减速度上限逼近目标
ramp.reset()                         # 安全路径：目标与下发值立即归零
ramp.idle                            # 目标与下发值都为 0 → 可释放租约
```

## 行为要点

- 身份解析：`ROBOT_ID`/`ROS_DOMAIN_ID` 环境变量优先，回退 topic 后缀；不硬编码机器人编号。
- 租约期间厂商键带 500 ms TTL，独立线程每 100 ms 以 Lua 原子续租；崩溃/断电后键自动过期，机械臂安全零流自动恢复。
- owner-token 键保证同一时刻只有一个控制中心实例持有；释放只认 token。
- 任何 Redis 异常 fail-closed：`acquire()` 返回 `False`，调用方必须只发零速并提示。
- 去抖只作用于“按键释放”；X 键、失焦、切页、关闭与显式急停一律立即归零，不走延迟。
- 重复 release 不会延长截止时间，避免虚假重复无限推迟停车。
- 平滑只作用于**人工键盘遥控**：键盘只写 `target`，周期 tick（导航页心跳线程 /
  建图页 velocity_timer）把 `output` 按 `accel/decel * dt` 逼近目标，因此
  Nav2 自动导航、VR 遥操、厂商驱动都不经过这条路径，行为不变。
- 按轴独立限制：`linear_accel`/`linear_decel` 作用于前后与左右平移，
  `angular_accel`/`angular_decel` 作用于转向；两者互不影响。
- 换向（W→S、A→D）按“减速”处理，必然经过 0 再反向加速，不会瞬时翻符号。
- 正常松键只把 `target` 置零 → 平滑减速；X 键、失焦、切页、关窗、退出、租约丢失
  走 `reset()` → 立即归零，安全语义未改变。
- 参数集中在 `control_center_chassis_guard/chassis_teleop_smoothing.yaml`，两个页面共用一份。

## 诊断（可选，默认关闭）

```bash
export CONTROL_CENTER_KEYDIAG=1
# 可选：export CONTROL_CENTER_KEYDIAG_PATH=/tmp/keydiag.log

export CONTROL_CENTER_VELDIAG=1
```

`CONTROL_CENTER_KEYDIAG` 按 JSON 行记录 press/release/expire/focus_out/close 与单调
时间戳，用于在真机上区分“自动重复对”与“焦点抖动”两类触发源。

`CONTROL_CENTER_VELDIAG` 按 JSON 行记录每次**下发速度发生变化**时的
`t / target_linear_x / target_linear_y / target_angular_z / output_linear_x /
output_linear_y / output_angular_z`，速度稳定时不写行，因此既能看全整条斜坡，
也不会持续刷屏。诊断文件固定落在系统临时目录
（`control_center_veldiag_<tag>_<pid>.log`），路径不接受外部输入，`tag` 已做字符过滤。
两者默认完全不写文件、不影响运动逻辑。

## 测试

```bash
/usr/bin/python3 -m pytest test -q
```

- 单元测试使用内置 `FakeRedis` 与假时钟；
- `test_ui_key_debounce_integration.py` 通过真实 Qt 事件流（持续按住 + 周期性伪
  release/press）断言两个页面**不产生零插帧**，并覆盖真实释放与安全旁路；
- `test_ui_ramp_integration.py` 同样用真实 Qt 事件驱动两个页面，逐帧断言斜坡形状：
  起步不是阶跃、松键立即减速、单调降到 0、换向经过 0、快点按不窜车、长按能到目标
  速度、组合键两轴独立、X 键/失焦/切页/退出立即停车；
- `test_velocity_ramp.py` 覆盖限制选择（加速/减速/换向）、`MAX_STEP_DT` 卡顿保护、
  配置读取与诊断输出；
- 真实 Redis 集成测试使用隔离键前缀 `zcode_test_chassis_guard:*`（绝不触碰生产键）。
