#!/usr/bin/env python3

import rospy
from nav_msgs.srv import GetMap, GetMapRequest, GetMapResponse


def main():
    rospy.init_node("static_map_proxy")

    target_service = rospy.get_param("~target_service", "/slam_toolbox/dynamic_map")
    service_name = rospy.get_param("~service_name", "/static_map")

    rospy.loginfo("[static_map_proxy] waiting for target service %s", target_service)
    rospy.wait_for_service(target_service)
    client = rospy.ServiceProxy(target_service, GetMap)

    def _handle(_req):
        try:
            return client(GetMapRequest())
        except rospy.ServiceException as exc:
            rospy.logerr_throttle(
                2.0,
                "[static_map_proxy] target call failed (%s -> %s): %s",
                service_name,
                target_service,
                exc,
            )
            return GetMapResponse()

    rospy.Service(service_name, GetMap, _handle)
    rospy.loginfo(
        "[static_map_proxy] forwarding %s -> %s", service_name, target_service
    )
    rospy.spin()


if __name__ == "__main__":
    main()
