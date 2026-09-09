# 307全局重定位：配对、抗遮挡和分散候选（2026-09-08）

仅修改 control_center_navigation 的自定义 cartographer_global_relocalization 节点、其配置及离线测试。控制中心与VR导航共用该节点；306/328未同步本次改动。未改Nav2避障、原始激光、AMCL、地图或航点。

## 已实现

1. 独立雷达回调组持续缓存每侧最新8帧。搜索定时器单独限流；按采集时间配对（≤0.10s），拒绝零时间戳、旧帧、重复帧和超过0.60s的数据。通过odom将两雷达对齐到同一base_link时刻。需要有效odom/TF；搜索期间移动超过4cm或0.03rad不发布旧位姿，提示停稳重试。没有添加历史地图位姿搜索先验。
2. 已有地图局部占据几何及激光连续线段作为结构支持，不是语义识别椅子/墙。结构权重有上限；所有采样回波保留。只有端点落在已知空闲区且后方存在地图结构的疑似遮挡，权重降到0.45，而非删除。使用匹配比例、多方向覆盖、连续墙面法向信息验证，避免单墙/平行墙高分误定位。上述处理仅供搜索评分，不发布过滤后的避障激光。
3. 在全局截取前按1m区域/朝向保留候选，每个区域最多2个种子，再跨区域去重选40个。每个精搜索种子重新用同一批240点评分；初始粗分数不继承。双线性距离场上的有界连续坐标优化进一步细化（≤24次、相对精搜索解≤10cm/2°）。仍比较远处分立的次优解；三个独立新帧必须连续确认，失败、缺帧或TF异常打断计数。

主要文件：src/cartographer_global_relocalization.cpp、config/global_relocalization.yaml。
新增参数：scan_max_age_sec、odom_frame、minimum_matched_ratio=0.35、minimum_structural_ratio=0.15、minimum_matched_sectors=3、minimum_normal_diversity=0.06、occluded_point_weight=0.45、candidate_region_m=1.0。原min_score=0.52、min_score_margin=0.03保留，但新旧分数公式不同，不能直接当作同一置信度比较。

## 验证与限制

- CTest离线回归包含22条运行断言：遮挡前后、错误位姿、单墙、重复柱点、亚栅格细化、同点集评分、相同房间歧义、缓存配对、旧帧/重复帧、完整三帧确认和坏帧重置。使用ROS_DOMAIN_ID=88与合成TF，不向真机发布定位或运动命令。
- 只读录制307约12秒：地图1帧、前后雷达各117帧及TF/odom。抽取3对采集时间相差17～22ms的雷达，同批点分别运行旧/新算法。
- 初轮实录搜索耗时：旧0.40～0.42s，新1.04～1.13s；每次搜索只在重定位启用期间运行。新候选的帧间波动较小，但这不是绝对误差验证。
- 当前实录样本虽有约64%～66%回波匹配，墙面法向独立性约0.03，低于0.06门槛；新版拒绝这类主要平行结构证据，不以高总分强行成功。
- 尚无独立真值和真实“加椅子前后”成对录包，不能宣称现场正确率、错误率或绝对精度已验收。合成场景误差不可作为真机精度指标。完全遮挡或相似走廊仍可能需要人工移动到能看到墙角的位置并停稳重试。
- 仅完成前三项；没有实现历史位置优先/局部到全局扩展，也没有更改导航避障。

## 备份与生效

2026-09-08 启动入口修复：MultiThreadedExecutor只保存节点弱引用，不能直接把make_shared临时对象传入add_node后丢弃。main中必须持有node直到spin结束，否则会出现进程仍在但地图订阅、重定位服务都不存在。已增加entrypoint_smoke.py，通过真实可执行文件、隔离Domain 88验证连续两次重试服务及状态回调；旧可执行文件复现失败，新构建和安装文件均通过，算法22断言回归也通过。此次修复前源码备份：/home/ubuntu/ros2_ws/migration_backups/relocal307_entrypoint_20260908。没有重启真机导航或触发真机定位。

完整原版本：/home/ubuntu/ros2_ws/migration_backups/relocal307_robust_Lgbgrjmi
含source_before.tgz、install_before.tgz、observation_bag、fixtures、regression.log。当前进程不重启；安全退出并重新打开307控制中心导航或~/vr307nav后生效。

回退时先正常关闭导航，再从source_before.tgz恢复该包的src/cartographer_global_relocalization.cpp、config/global_relocalization.yaml、CMakeLists.txt，并重新编译control_center_navigation。地图和航点无需回退。旧构建产物也保存在install_before.tgz。
