import sys
# 显示当前运行路径
sys.path.append('/home/lza/code/04_uncalibrate_robot/01_calibrate_robot/build/modules/pybind')
import robot

robot.init()

status=robot.get_status()
print(status)
robot.log_out()
# import robot
# robot.main()