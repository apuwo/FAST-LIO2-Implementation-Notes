import rospy
from nav_msgs.msg import Odometry
def callback(msg):
# 根据消息类型提取数据
# 对于 nav_msgs/Odometry:
	timestamp = msg.header.stamp.to_sec()
	pos = msg.pose.pose.position
	ori = msg.pose.pose.orientation


# 格式化为 TUM 标准格式 (保留6位小数)
	data_line = "{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(
	timestamp, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w)
# 写入文件
	with open("ground_truth_tum.txt", "a") as f:
		f.write(data_line)
if __name__ == "__main__":
	rospy.init_node("tum_recorder_groundtruth")
# 修改为你实际的话题名称
	rospy.Subscriber("/quad_0/lidar_slam/odom", Odometry, callback)
	print("开始记录轨迹到 ground_truth_tum.txt ...")
	rospy.spin()
