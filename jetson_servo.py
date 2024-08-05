import rospy
from mavros_msgs.srv import CommandLong

def send_set_servo_command(servo_number, pwm_value):
        rospy.wait_for_service('/mavros/cmd/command')
    # try:
        command_service = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        response = command_service(
            False,        # broadcast
            183,          # MAV_CMD_DO_SET_SERVO
            0,            # confirmation
            servo_number, # param1: Servo number (1 for first servo, 2 for second, etc.)
            pwm_value,    # param2: PWM value (e.g., 1100 to 1900)
            0,            # param3
            0,            # param4
            0,            # param5
            0,            # param6
            0             # param7
        )
        return response.success
    # except rospy.ServiceException as e:
    #     rospy.logerr("Service call failed: %s" % e)
    #     return False

def delivery(servo_channel=7, pwm_values=[2000, 800]):
    for pwm in pwm_values:
        send_set_servo_command(servo_channel, pwm)
        rospy.Rate(20).sleep()

# Inicializa o nó ROS
rospy.init_node('servo_control_node')

# Testando os comandos de servo
send_set_servo_command(7, 2000)
rospy.logwarn(f"Pós chamada")
send_set_servo_command(7, 900)
send_set_servo_command(7, 1900)
