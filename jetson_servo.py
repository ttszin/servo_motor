import rospy
from mavros_msgs.msg import OverrideRCIn

def set_servo_via_rc_override(channel, pwm_value):
    pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
    rc_override = OverrideRCIn()
    rc_override.channels = [0] * 8  # Inicializa todos os canais com 0 (nenhum override)
    rc_override.channels[channel - 1] = pwm_value  # Define o PWM no canal especificado
    pub.publish(rc_override)
    rospy.sleep(1)  # Aguarda 1 segundo para garantir que o comando seja enviado

# Inicializa o nó ROS
rospy.init_node('servo_control_node')

# Testando os comandos de servo
set_servo_via_rc_override(7, 2000)
rospy.sleep(1)
set_servo_via_rc_override(7, 800)
rospy.sleep(1)
set_servo_via_rc_override(7, 2000)
