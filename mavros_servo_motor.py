import rospy
from mavros_msgs.srv import CommandLong

rospy.init_node('set_servo_pwm')

rospy.wait_for_service('/mavros/cmd/command')
try:
    command_service = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
    response = command_service(
        False,  # broadcast
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
        0,  # confirmation
        channel,  # param1 (servo number)
        pwm_value,  # param2 (PWM value)
        0, 0, 0, 0, 0, 0  # param3 ~ param7 (not used)
    )
    if response.success:
        print("Comando enviado com sucesso!")
    else:
        print("Falha ao enviar o comando.")
except rospy.ServiceException as e:
    print("Falha ao chamar o serviço: %s" % e)
