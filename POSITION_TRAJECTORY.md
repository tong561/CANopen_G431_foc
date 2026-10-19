# 位置梯形轨迹

APOS/RPOS 保留原格式和含义。USB_PositionTarget 是最终累计机械位置，16384 count/圈。RPOS 以指令接收时的实际位置为基准。

foc.h 参数：
- FOC_POS_TRAJ_MAX_RPM：1000 rpm，且不超过 POS_PI.out_limit。
- FOC_POS_TRAJ_ACCEL_RPM_S：2000 rpm/s。
- FOC_POS_TRAJ_DECEL_RPM_S：2000 rpm/s。

1 kHz 更新。短距离自动成为三角速度轨迹，长距离为加速、匀速、减速。运动中换目标保留参考位置和速度，反向时先制动；若新目标在当前刹车距离以内，参考允许越过新目标再返回，不瞬移。进入位置模式以实际位置、测量速度初始化，退出/停机/校准时复位。

位置控制采用轨迹速度前馈、位置 PID 反馈及条件积分抗饱和。原 PPID 单位和积分上限保留。轨迹约束作用于规划参考，实际速度和扭矩仍受控制器及负载影响。

观察变量：
- POS_PI.pos：实际累计位置。
- USB_PositionTarget：最终位置。
- FOC_PositionReference：规划中间位置（double，打印时转 float）。
- FOC_PositionVelocityRPM：规划速度。
- error_pos：规划位置减实际位置。

现有 USB 打印通道未改变。my_mian.c 的 FOC_POSITION_AUTO_TEST 默认 0，避免原自动测试覆盖 USB 目标。

例：静止起步 APOS,1638400, 为100圈，默认参数规划约6.5秒；1圈规划峰值约346 rpm，为三角速度轨迹。

验证：提取实际 C 规划器进行主机仿真，覆盖正反 1/100 count、1/4/100圈，以及运行中改近目标、改到当前位置、反向和缩短目标。检查每毫秒速度增量及最大速度、最终位置和停止速度。电机带载效果需要烧录后实测。
