import rospy
from mavros_msgs.srv import CommandLong,CommandLongRequest,CommandBool

def set_custom_mode(custom_mode: str = "GUIDED") -> str:
        """This method set a custom mode to UAV

        Params:
        --
        custom_mode: The custom mode string that specifies the desired mode of the UAV, allowing you to set a specific behavior or flight mode defined in the flight controller or autopilot software.

        Possible custom modes:
        [STABILIZE, ACRO, ALT_HOLD, AUTO, GUIDED, LOITER, RTL, CIRCLE, POSITION, LAND, OF_LOITER, DRIFT, SPORT, FLIP, AUTOTUNE, POSHOLD, BRAKE, THROW, AVOID_ADSB, GUIDED_NOGPS]

        Returns:
        --
        response.mode_sent: returns true if the service worked correctly.
        """
        rospy.wait_for_service("/mavros/set_mode", timeout=60)
        try:
            rospy.ServiceProxy("/mavros/set_mode", CommandBool)
            response = (0, custom_mode)
            return response.mode_sent
        except rospy.ServiceException as service_exception:
            raise rospy.ServiceException from service_exception

if __name__ == "__main__":
    set_custom_mode()