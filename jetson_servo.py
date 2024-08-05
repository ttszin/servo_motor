import rospy
from mavros_msgs.srv import CommandLong, CommandLongRequest

def send_set_servo_command(servo_number, pwm_value):
    rospy.wait_for_service('/mavros/cmd/command')
    try:
        command_service = rospy.ServiceProxy('/mavros/cmd/command', CommandLongRequest)
        request = CommandLongRequest(
            broadcast=False,
            command=183,  # MAV_CMD_DO_SET_SERVO
            confirmation=0,
            param1=servo_number,  # Servo number (1 for first servo, 2 for second, etc.)
            param2=pwm_value,     # PWM value (e.g., 1100 to 1900)
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )
        response = command_service(request)
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False
        

    def delivery(self, servo_channel=7, pwm_values=[2000, 700]):
        for pwm in pwm_values:
            self.send_set_servo_command(servo_channel, pwm)
            rospy.Rate(20).sleep()


# Inicializa o nó ROS
rospy.init_node('servo_control_node')

# Testando os comandos de servo
send_set_servo_command(7, 2000)
rospy.sleep(1)
send_set_servo_command(7, 800)
rospy.sleep(1)
send_set_servo_command(7, 2000)