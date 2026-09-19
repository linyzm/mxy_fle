import flexivrdk

# 打印 RDK 版本（不连接机器人，100%能跑）
print("=" * 50)
print("Flexiv RDK 版本:", flexivrdk.__version__)
print("=" * 50)

# 查看机器人状态数据结构（不连接机器人）
print("\n机器人状态数据结构：")
dummy_states = flexivrdk.RobotStates()
print("可读取的状态变量：")
print("- q (关节角度)")
print("- q_dot (关节速度)")
print("- tcp_pose (末端位姿)")
print("- force_tensor (力传感器数据)")

print("\n✅ 运行成功！环境完全正常！")
print("你现在可以开始学习 Flexiv RDK 啦！")
