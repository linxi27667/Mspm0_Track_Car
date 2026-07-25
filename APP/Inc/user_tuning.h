/**
 * @file    user_tuning.h
 * @brief   全局用户调参默认值 — 固件侧唯一定义源。
 *
 *          所有 USER_* 宏在此文件集中定义。track.h / app_encoder_motor.h
 *          仅保留 TRACK_* / MOTOR_* 别名，不再保留 #ifndef 回退默认值。
 *          main.c 不再重复定义。
 *
 *          远程可调参数的 min/max/default 同时由 host/param_schema.py
 *          驱动，tests/test_param_schema.py 验证两者默认值一致。
 *          最终比赛档值唯一落在本文件，RACE 构建直接编译生效。
 *
 * ====== 国一/省一方案学习总结(2024电赛H题) ======
 * 1. 陀螺仪: 国一方案都用JY61P/HWT101数字陀螺仪(直接输出角度,无零漂)
 *    我们用IMU660RB需积分,需开机静止校准+PID稳定
 * 2. 状态机+航向锁定: 每段路锁定目标航向角,陀螺仪PID走直线
 * 3. 循迹时也保持速度闭环: 灰度只输出差速,速度环保证车速稳定
 * 4. 位置前馈: 编码器累计脉冲数判断到达点(我们用圈数+超时)
 * 5. 速度环只用P就够: 省一方案验证P环足够,KI=0避免积分饱和
 * 6. 差速范围: 省一方案离散差速最大±28,我们OLIMIT=30起步
 * 7. 丢线处理: 锁定航向角直行+编码器测距判断,超时停车
 */
#ifndef USER_TUNING_H
#define USER_TUNING_H

/* ==================================================================
 * ====== 1. 循迹线PID (主循迹,输入灰度误差,输出转向量) ======
 * 调参方法:
 *   1. 先只调KP: 从小值(2.0)开始,车能跟线但反应慢→加大KP
 *   2. 车甩出弯道→KP太大,减小
 *   3. 车左右震荡→加KD(从0.5开始),KD约为KP的1/5~1/3
 *   4. 直线有静差→加KI(从0.1开始),但通常循迹KI=0
 *   5. 转向量过大→减小OLIMIT
 * 参考省一方案: 离散差速最大±28,我们用连续PID,OLIMIT建议30起步
 * 参考wandao方案: KP=5 KI=0 KD=3(注意KD=KP*0.6,偏大抑制震荡)
 * 综合建议: KP=5 KD=1(KD=KP/5) 起步,实测调整
 * ================================================================== */
#ifndef USER_LINE_PID_KP
#define USER_LINE_PID_KP           3.5f    /* 实车低速档: 降低跨线后的反向摆动 */
#endif
#ifndef USER_LINE_PID_KI
#define USER_LINE_PID_KI           0.0f    /* 积分增益(循迹通常为0,国一方案验证) */
#endif
#ifndef USER_LINE_PID_KD
#define USER_LINE_PID_KD           1.5f    /* 增加误差变化阻尼，抑制左右摆动 */
#endif
#ifndef USER_LINE_PID_ILIMIT
#define USER_LINE_PID_ILIMIT       50.0f   /* 积分限幅 */
#endif
#ifndef USER_LINE_PID_OLIMIT
#define USER_LINE_PID_OLIMIT       30.0f   /* 输出限幅(省一参考:差速最大±28) */
#endif
#ifndef USER_LINE_PID_DERIV_ALPHA
#define USER_LINE_PID_DERIV_ALPHA  0.6f    /* 不完全微分系数(0~1,越大滤波越弱) */
#endif
/* 国一方案核心: LINE_FOLLOW 陀螺仪角速度阻尼 (直行稳/弯道灵)
 * 实车极性: 左转gyro_z>0、左转差速turn<0，因此阻尼取正号：
 * 公式: turn_output = line_pid + GYRO_DAMP * gyro_z
 * 调参: 直道抖动→加大(0.15→0.3), 弯道跟踪迟钝→减小(0.15→0.08) */
#ifndef USER_LINE_FOLLOW_GYRO_DAMP
#define USER_LINE_FOLLOW_GYRO_DAMP 0.25f   /* 实车直线角速度阻尼 */
#endif
#ifndef USER_LINE_RECOVERY_MAX_ERROR
#define USER_LINE_RECOVERY_MAX_ERROR 1.5f  /* 丢线/无效帧沿末次方向低增益找线 */
#endif
#ifndef USER_LINE_RECOVERY_KP
#define USER_LINE_RECOVERY_KP        1.5f  /* 冻结灰度PID后仅保留末次方向比例项 */
#endif
#ifndef USER_LINE_RECOVERY_HEADING_KP
#define USER_LINE_RECOVERY_HEADING_KP 0.30f /* 锁定最后有效鱼眼帧的航向 */
#endif
#ifndef USER_LINE_RECOVERY_HEADING_KD
#define USER_LINE_RECOVERY_HEADING_KD 0.06f /* 角速度阻尼，避免找线时左右摆动 */
#endif
#ifndef USER_LINE_RECOVERY_HEADING_LIMIT
#define USER_LINE_RECOVERY_HEADING_LIMIT 6.0f
#endif
#ifndef USER_LINE_RECOVERY_TURN_LIMIT
#define USER_LINE_RECOVERY_TURN_LIMIT 10.0f
#endif
#ifndef USER_LINE_RECOVERY_SPEED
#define USER_LINE_RECOVERY_SPEED     12     /* 丢线后立即降到可控低速 */
#endif
#ifndef USER_LINE_LOST_RECOVERY_MS
#define USER_LINE_LOST_RECOVERY_MS   600u  /* 沿最后方向主动找线，超时后再锁定停车 */
#endif

/* ==================================================================
 * ====== 2. 速度PID (循迹与旋转共用左右独立闭环) ======
 * 调参方法:
 *   1. 空转时PWM很高(如90)是正常的(编码器读数≈0,PID拼命加PWM)
 *   2. 接轮子后车不动→加大KP
 *   3. 车速震荡→加KD或减小KP
 *   4. 速度跟不上→加大OLIMIT或减小INTEGRAL_SEP
 * 速度比例尺(SPEED_SCALE): 国一方案必须左右分别标定!
 *   方法1(自动): 悬空车轮, K3长按触发 App_Motor_Calibrate_Scale(), RTT打印scale值
 *   方法2(手动): 设PWM=50,测encoder_speed, scale = encoder_speed / 50
 *   注意: 左右均为GPIO 4x解码，仍需分别标定以覆盖机械与安装差异
 * 速度环只用P就够(省一方案验证),KI=0避免积分饱和
 * ================================================================== */
#ifndef USER_MOTOR_SPEED_PID_KP
#define USER_MOTOR_SPEED_PID_KP    0.55f   /* 5ms encoder P loop; filtered feedback */
#endif
#ifndef USER_MOTOR_SPEED_PID_KI
#define USER_MOTOR_SPEED_PID_KI    0.0f    /* 速度PID积分增益(省一验证:P环足够) */
#endif
#ifndef USER_MOTOR_SPEED_PID_KD
#define USER_MOTOR_SPEED_PID_KD    0.0f    /* 速度PID微分增益 */
#endif
#ifndef USER_MOTOR_SPEED_PID_ILIMIT
#define USER_MOTOR_SPEED_PID_ILIMIT 30.0f  /* 速度PID积分限幅 */
#endif
#ifndef USER_MOTOR_SPEED_PID_OLIMIT
#define USER_MOTOR_SPEED_PID_OLIMIT 20.0f  /* correction limit before feed-forward */
#endif
#ifndef USER_MOTOR_SPEED_PID_INTEGRAL_SEP
#define USER_MOTOR_SPEED_PID_INTEGRAL_SEP 15.0f /* 积分分离阈值(误差>此值不累加I) */
#endif
#ifndef USER_MOTOR_SPEED_SCALE_LEFT
#define USER_MOTOR_SPEED_SCALE_LEFT   0.367f /* raised-wheel PWM20/30/40 regression */
#endif
#ifndef USER_MOTOR_SPEED_SCALE_RIGHT
#define USER_MOTOR_SPEED_SCALE_RIGHT  0.375f /* raised-wheel PWM20/30/40 regression */
#endif
#ifndef USER_MOTOR_SPEED_FILTER_ALPHA
#define USER_MOTOR_SPEED_FILTER_ALPHA 0.20f /* about 20ms time constant at 5ms/tick */
#endif
#ifndef USER_MOTOR_SPEED_START_THRESHOLD
#define USER_MOTOR_SPEED_START_THRESHOLD 2.0f
#endif
#ifndef USER_MOTOR_SPEED_MIN_DRIVE_PWM
#define USER_MOTOR_SPEED_MIN_DRIVE_PWM 24  /* measured motors start around PWM=25 */
#endif

/* ==================================================================
 * ====== 3. 循迹策略参数（数值均为PWM百分比） ======
 * 调参方法:
 *   1. 车速太快冲出弯道→减小BASE_SPEED或SPEED_STRAIGHT
 *   2. 直道不够快→加大SPEED_STRAIGHT
 *   3. 弯道转不过→减小SPEED_CURVE或加大MOTOR_INNER_MIN
 *   4. 急转时内侧轮停转→减小MOTOR_INNER_MIN(最低15)
 * 省一方案: 单圈15秒,说明速度中等,我们用40起步
 * ================================================================== */
#ifndef USER_TRACK_BASE_SPEED
#define USER_TRACK_BASE_SPEED      26      /* 实车循迹起步档 */
#endif
#ifndef USER_TRACK_SPEED_STRAIGHT
#define USER_TRACK_SPEED_STRAIGHT  30      /* 实车直道提速档 */
#endif
#ifndef USER_TRACK_SPEED_CURVE
#define USER_TRACK_SPEED_CURVE     24      /* 普通偏线减速档 */
#endif
#ifndef USER_TRACK_SPEED_RAMP_STEP
#define USER_TRACK_SPEED_RAMP_STEP 2       /* 速度渐变步进(避免突变抖动) */
#endif
#ifndef USER_TRACK_MOTOR_INNER_MIN
#define USER_TRACK_MOTOR_INNER_MIN 24      /* 循迹轮最小有效PWM */
#endif
#ifndef USER_TRACK_CIRCLES
#define USER_TRACK_CIRCLES         1       /* 目标圈数 */
#endif
/* 国一方案: 连续曲率变速 (替代二档跳变)
 * |error|=0→SPEED_STRAIGHT, |error|>=此值→SPEED_CURVE, 中间线性插值 */
#ifndef USER_TRACK_CURVE_ERROR_MAX
#define USER_TRACK_CURVE_ERROR_MAX 2.5f    /* 误差达此值时降到弯道速度 */
#endif
#ifndef USER_TRACK_SPEED_PID_ENABLE
#define USER_TRACK_SPEED_PID_ENABLE 1       /* 循迹默认启用左右独立轮速闭环 */
#endif
/* ==================================================================
 * ====== 4. 丢线/直角弯参数 ======
 * 调参方法:
 *   1. 拐角检测不到→确认近期有连续2帧居中，RATHR保持4
 *   2. 普通宽线被误判为拐角→将RATHR按整数逐步调到5~6
 * 逆时针方形赛道模式: 近期居中后，任意非全黑且达到RATHR的展开
 * 图案均作为角点证据；转向固定为左转，mask不参与方向决策。
 * ================================================================== */
#ifndef USER_RIGHT_ANGLE_THRESHOLD
#define USER_RIGHT_ANGLE_THRESHOLD 6.0f    /* 12路角点候选最少黑通道数 */
#endif
#ifndef USER_RIGHT_ANGLE_CONFIRM_FRAMES
#define USER_RIGHT_ANGLE_CONFIRM_FRAMES 1  /* 严格角点图案单帧触发；仍需2帧居中历史 */
#endif
#ifndef USER_RIGHT_ANGLE_WHITE_FALLBACK_FRAMES
#define USER_RIGHT_ANGLE_WHITE_FALLBACK_FRAMES 2u /* 居中后连续全白兜底确认帧数 */
#endif
#ifndef USER_RIGHT_ANGLE_WHITE_RECENT_MS
#define USER_RIGHT_ANGLE_WHITE_RECENT_MS 100u /* 全白兜底必须紧跟正常居中线型 */
#endif
#ifndef USER_RIGHT_ANGLE_ARM_DELAY
#define USER_RIGHT_ANGLE_ARM_DELAY  200     /* 启动后短暂稳定，避免漏掉靠近起点的首个角 */
#endif
#ifndef USER_RIGHT_ANGLE_TIMEOUT
#define USER_RIGHT_ANGLE_TIMEOUT   3500    /* PIVOT/CAPTURE阶段重新校正周期，不触发停车 */
#endif
#ifndef USER_RIGHT_ANGLE_TARGET_ANGLE
#define USER_RIGHT_ANGLE_TARGET_ANGLE    90.0f /* 逆时针方形赛道固定左转角度 */
#endif
#ifndef USER_ENCODER_LEFT_COUNTS_PER_MM
#define USER_ENCODER_LEFT_COUNTS_PER_MM  0.0f  /* 必须用左轮实测，不允许猜测 */
#endif
#ifndef USER_ENCODER_RIGHT_COUNTS_PER_MM
#define USER_ENCODER_RIGHT_COUNTS_PER_MM 0.0f  /* 必须用右轮实测，不允许与左轮共用 */
#endif
#ifndef USER_SENSOR_TO_AXLE_MM
#define USER_SENSOR_TO_AXLE_MM           0.0f  /* 探头检测中心到驱动轮轴实测距离 */
#endif
#ifndef USER_RIGHT_ANGLE_DETECT_COMP_MM
#define USER_RIGHT_ANGLE_DETECT_COMP_MM  0.0f  /* 单帧确认与制动造成的补偿距离 */
#endif
#ifndef USER_RIGHT_ANGLE_APPROACH_SPEED
#define USER_RIGHT_ANGLE_APPROACH_SPEED  28    /* 闭环前探轮速，兼顾距离与停车冲击 */
#endif
#ifndef USER_RIGHT_ANGLE_APPROACH_FALLBACK_MS
#define USER_RIGHT_ANGLE_APPROACH_FALLBACK_MS 750u /* 最新整圈日志基础上再增加50ms前探 */
#endif
#ifndef USER_RIGHT_ANGLE_STOP_MS
#define USER_RIGHT_ANGLE_STOP_MS         60u   /* 原地转向前短暂停稳 */
#endif
#ifndef USER_RIGHT_ANGLE_RETRIGGER_DELAY
#define USER_RIGHT_ANGLE_RETRIGGER_DELAY 300u /* 出弯后禁止立即重复计角 */
#endif
#ifndef USER_RIGHT_ANGLE_TURN_LIMIT
#define USER_RIGHT_ANGLE_TURN_LIMIT      18.0f /* 循迹差速限幅 */
#endif
#ifndef USER_RIGHT_ANGLE_DONE_ERROR
#define USER_RIGHT_ANGLE_DONE_ERROR       1.5f /* 与实测90度原地旋转使用相同误差窗 */
#endif
#ifndef USER_RIGHT_ANGLE_DONE_RATE
#define USER_RIGHT_ANGLE_DONE_RATE        6.0f /* 到位后必须基本静止(deg/s) */
#endif
#ifndef USER_RIGHT_ANGLE_DONE_FRAMES
#define USER_RIGHT_ANGLE_DONE_FRAMES     20u   /* 连续100ms稳定后才开始寻线 */
#endif
#ifndef USER_RIGHT_ANGLE_SEEK_MIN_TURN_DEG
#define USER_RIGHT_ANGLE_SEEK_MIN_TURN_DEG 85.0f /* 完整转稳后寻线阶段的姿态保护 */
#endif
#ifndef USER_RIGHT_ANGLE_CAPTURE_SPEED
#define USER_RIGHT_ANGLE_CAPTURE_SPEED    18   /* 陀螺仪转稳后向前寻线 */
#endif
#ifndef USER_RIGHT_ANGLE_CAPTURE_TURN_LIMIT
#define USER_RIGHT_ANGLE_CAPTURE_TURN_LIMIT 8.0f
#endif
#ifndef USER_RIGHT_ANGLE_CAPTURE_LINE_LIMIT
#define USER_RIGHT_ANGLE_CAPTURE_LINE_LIMIT 3.0f /* 灰度项不得压过航向控制权 */
#endif
#ifndef USER_RIGHT_ANGLE_CAPTURE_HEADING_KP
#define USER_RIGHT_ANGLE_CAPTURE_HEADING_KP 0.20f /* 捕线时航向仅作辅助 */
#endif
#ifndef USER_RIGHT_ANGLE_CAPTURE_HEADING_KD
#define USER_RIGHT_ANGLE_CAPTURE_HEADING_KD 0.04f
#endif
#ifndef USER_RIGHT_ANGLE_CAPTURE_HEADING_LIMIT
#define USER_RIGHT_ANGLE_CAPTURE_HEADING_LIMIT 3.0f
#endif
#ifndef USER_RIGHT_ANGLE_CAPTURE_CONFIRM_FRAMES
#define USER_RIGHT_ANGLE_CAPTURE_CONFIRM_FRAMES 3u
#endif
#ifndef USER_RIGHT_ANGLE_CAPTURE_LINE_ERROR
#define USER_RIGHT_ANGLE_CAPTURE_LINE_ERROR 3.5f /* 接受边缘正常线型并快速交回完整循迹PID */
#endif
#ifndef USER_RIGHT_ANGLE_CAPTURE_MAX_ERROR
#define USER_RIGHT_ANGLE_CAPTURE_MAX_ERROR 12.0f /* CORNER_DONE时最大航向误差 */
#endif
#ifndef USER_RIGHT_ANGLE_CAPTURE_REPIVOT_ERROR
#define USER_RIGHT_ANGLE_CAPTURE_REPIVOT_ERROR 25.0f /* 超窗则回原地航向闭环 */
#endif
/* Rotation/heading outer loop: angle P + measured yaw-rate damping. Its
 * output is a wheel-speed target; the encoder P loop below it produces PWM. */
#ifndef USER_ROTATE_PID_KP
#define USER_ROTATE_PID_KP              0.80f  /* wheel-speed / degree */
#endif
#ifndef USER_ROTATE_PID_KD_RATE
#define USER_ROTATE_PID_KD_RATE         0.08f  /* wheel-speed / (degree/second) */
#endif
#ifndef USER_ROTATE_PID_MAX_SPEED
#define USER_ROTATE_PID_MAX_SPEED       24.0f
#endif
#ifndef USER_ROTATE_PID_MIN_SPEED
#define USER_ROTATE_PID_MIN_SPEED       8
#endif
#ifndef USER_ROTATE_PID_COAST_OUTPUT
#define USER_ROTATE_PID_COAST_OUTPUT    4.0f
#endif
#ifndef USER_ROTATE_PID_FINE_ZONE_DEG
#define USER_ROTATE_PID_FINE_ZONE_DEG    10.0f
#endif
#ifndef USER_ROTATE_PID_BLEND_ZONE_DEG
#define USER_ROTATE_PID_BLEND_ZONE_DEG   25.0f
#endif
#ifndef USER_ROTATE_PID_FINE_MAX_SPEED
#define USER_ROTATE_PID_FINE_MAX_SPEED    9.0f
#endif
#ifndef USER_ROTATE_PID_FINE_MIN_SPEED
#define USER_ROTATE_PID_FINE_MIN_SPEED   3
#endif
#ifndef USER_ROTATE_DONE_ERR_DEG
#define USER_ROTATE_DONE_ERR_DEG          1.5f
#endif
#ifndef USER_ROTATE_SPEED_SLEW_STEP
#define USER_ROTATE_SPEED_SLEW_STEP      1
#endif
#ifndef USER_ROTATE_SETTLE_FRAMES
#define USER_ROTATE_SETTLE_FRAMES        20u  /* 100ms at the 5ms control tick */
#endif
#ifndef USER_ROTATE_STALL_RATE_DEG_S
#define USER_ROTATE_STALL_RATE_DEG_S      2.0f /* 近目标静止判据 */
#endif
#ifndef USER_ROTATE_STALL_MIN_ERROR_DEG
#define USER_ROTATE_STALL_MIN_ERROR_DEG   6.0f /* 小角度命令不启用破静摩擦助推 */
#endif
#ifndef USER_ROTATE_STALL_MAX_ERROR_DEG
#define USER_ROTATE_STALL_MAX_ERROR_DEG 180.0f /* 全角度静止时允许一次破静摩擦助推 */
#endif
#ifndef USER_ROTATE_STALL_ENCODER_IDLE_MAX
#define USER_ROTATE_STALL_ENCODER_IDLE_MAX 1   /* 容忍霍尔边沿量化抖动 */
#endif
#ifndef USER_ROTATE_STALL_CONFIRM_FRAMES
#define USER_ROTATE_STALL_CONFIRM_FRAMES 20u   /* 双编码器+角速度静止100ms */
#endif
#ifndef USER_ROTATE_STALL_KICK_SPEED
#define USER_ROTATE_STALL_KICK_SPEED    32     /* 落地实测可克服轮胎静摩擦 */
#endif
#ifndef USER_ROTATE_STALL_KICK_HOLD_FRAMES
#define USER_ROTATE_STALL_KICK_HOLD_FRAMES 40u /* 启动助推保持200ms，确保两轮均越过静摩擦 */
#endif
#ifndef USER_ROTATE_STALL_RELEASE_RATE_DEG_S
#define USER_ROTATE_STALL_RELEASE_RATE_DEG_S 4.0f /* 明确恢复转动后才退出助推 */
#endif
#ifndef USER_ROTATE_STALL_RELEASE_ENCODER_SPEED
#define USER_ROTATE_STALL_RELEASE_ENCODER_SPEED 1 /* |encoder_speed|>1视为恢复 */
#endif
#ifndef USER_ROTATE_WRONG_WAY_RATE_DEG_S
#define USER_ROTATE_WRONG_WAY_RATE_DEG_S 15.0f /* 明确反向旋转才触发，滤除静态噪声 */
#endif
#ifndef USER_ROTATE_WRONG_WAY_FRAMES
#define USER_ROTATE_WRONG_WAY_FRAMES       20u /* 连续100ms反向立即停车 */
#endif
#ifndef USER_ROTATE_FEEDBACK_GRACE_MS
#define USER_ROTATE_FEEDBACK_GRACE_MS      150u /* 启动助推后检查双轮累计反馈 */
#endif
#ifndef USER_ROTATE_FEEDBACK_MIN_COUNTS
#define USER_ROTATE_FEEDBACK_MIN_COUNTS      4  /* 低于4个4x边沿视为该轮未启动 */
#endif
#ifndef USER_ROTATE_TIMEOUT_BASE_MS
#define USER_ROTATE_TIMEOUT_BASE_MS     1000u
#endif
#ifndef USER_ROTATE_TIMEOUT_PER_DEG_MS
#define USER_ROTATE_TIMEOUT_PER_DEG_MS 120u
#endif

/* Current-heading hold uses the rotation PD settings above. */
#ifndef USER_HEADING_HOLD_DEADBAND_DEG
#define USER_HEADING_HOLD_DEADBAND_DEG   2.0f
#endif
#ifndef USER_HEADING_HOLD_SETTLE_RATE_DEG_S
#define USER_HEADING_HOLD_SETTLE_RATE_DEG_S  8.0f
#endif
#ifndef USER_HEADING_HOLD_MAX_SPEED
#define USER_HEADING_HOLD_MAX_SPEED       14.0f
#endif
#ifndef USER_HEADING_HOLD_MIN_SPEED
#define USER_HEADING_HOLD_MIN_SPEED         6
#endif
#ifndef USER_HEADING_HOLD_COAST_OUTPUT
#define USER_HEADING_HOLD_COAST_OUTPUT     1.5f
#endif

/* ==================================================================
 * ====== 远程可调参数范围 (与 host/param_schema.py 一致) ======
 * 用于 app_tuning.c 对下发参数做固件侧钳位，防止第三方客户端绕过主机校验。
 * tests/test_param_schema.py 验证这些范围与 param_schema.py 一致。
 * ================================================================== */
#define USER_PARAM_KP_MIN     0.0f
#define USER_PARAM_KP_MAX     50.0f
#define USER_PARAM_KI_MIN     0.0f
#define USER_PARAM_KI_MAX     5.0f
#define USER_PARAM_KD_MIN     0.0f
#define USER_PARAM_KD_MAX     20.0f
#define USER_PARAM_GDAMP_MIN  0.0f
#define USER_PARAM_GDAMP_MAX  1.0f
#define USER_PARAM_VBASE_MIN  0
#define USER_PARAM_VBASE_MAX  100
#define USER_PARAM_VSTR_MIN   0
#define USER_PARAM_VSTR_MAX   100
#define USER_PARAM_VCUR_MIN   0
#define USER_PARAM_VCUR_MAX   100
#define USER_PARAM_RATHR_MIN  5.0f
#define USER_PARAM_RATHR_MAX 10.0f

#define USER_PARAM_MKP_MIN    0.0f
#define USER_PARAM_MKP_MAX    5.0f
#define USER_PARAM_MKI_MIN    0.0f
#define USER_PARAM_MKI_MAX    1.0f
#define USER_PARAM_MKD_MIN    0.0f
#define USER_PARAM_MKD_MAX    5.0f
#define USER_PARAM_MSL_MIN    0.01f
#define USER_PARAM_MSL_MAX    2.0f
#define USER_PARAM_MSR_MIN    0.01f
#define USER_PARAM_MSR_MAX    2.0f

#endif /* USER_TUNING_H */
