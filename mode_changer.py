import rospy
from mavros_msgs.srv import SetMode, SetModeRequest

def set_custom_mode(custom_mode: str = "GUIDED") -> bool:
    """This method sets a custom mode to the UAV.

    Params:
    --
    custom_mode: The custom mode string that specifies the desired mode of the UAV, allowing you to set a specific behavior or flight mode defined in the flight controller or autopilot software.

    Possible custom modes:
    [STABILIZE, ACRO, ALT_HOLD, AUTO, GUIDED, LOITER, RTL, CIRCLE, POSITION, LAND, OF_LOITER, DRIFT, SPORT, FLIP, AUTOTUNE, POSHOLD, BRAKE, THROW, AVOID_ADSB, GUIDED_NOGPS]

    Returns:
    --
    response.mode_sent: returns True if the service worked correctly.
    """
    rospy.wait_for_service("/mavros/set_mode", timeout=60)
    try:
        set_mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        request = SetModeRequest()
        request.base_mode = 0
        request.custom_mode = custom_mode
        response = set_mode_service(request)
        return response.mode_sent
    except rospy.ServiceException as service_exception:
        rospy.logerr(f"Service call failed: {service_exception}")
        return False

if __name__ == "__main__":
    rospy.init_node('set_mode_node')
    result = set_custom_mode("GUIDED")
    if result:
        rospy.loginfo("Mode change successful!")
    else:
        rospy.logerr("Mode change failed.")
