# 云台下发指令（视觉 -> 电控）
bool control            # 是否锁定目标（true 时下位机执行跟随角度）
float64 yaw             # 云台 yaw 角（rad）
float64 pitch           # 云台 pitch 角（rad）
float64 distance        # 目标距离（m）
float64 vx              # 目标线速度 x（m/s，世界系）
float64 vy              # 目标线速度 y（m/s，世界系）
float64 vz              # 目标线速度 z（m/s，世界系）
float64 yaw_rate        # 视线角速度 yaw（rad/s）
float64 pitch_rate      # 视线角速度 pitch（rad/s）
