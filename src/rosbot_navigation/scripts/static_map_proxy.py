#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap, GetMapResponse


def main():
    rospy.init_node("static_map_proxy")

    target_service = rospy.get_param("~target_service", "/slam_toolbox/dynamic_map")
    service_name = rospy.get_param("~service_name", "/static_map")
    map_topic = rospy.get_param("~map_topic", "/map")
    use_target_service = rospy.get_param("~use_target_service", False)

    latest_map = {"msg": None}

    def _map_cb(msg):
        if msg.info.width > 0 and msg.info.height > 0:
            latest_map["msg"] = msg

    rospy.Subscriber(map_topic, OccupancyGrid, _map_cb, queue_size=1)

    client = None
    if use_target_service:
        rospy.loginfo(
            "[static_map_proxy] waiting for target service %s", target_service
        )
        rospy.wait_for_service(target_service)
        client = rospy.ServiceProxy(target_service, GetMap)

    def _handle(_req):
        if latest_map["msg"] is not None:
            return GetMapResponse(map=latest_map["msg"])

        if client is not None:
            try:
                resp = client()
                if resp.map.info.width > 0 and resp.map.info.height > 0:
                    return resp
                rospy.logwarn_throttle(
                    2.0,
                    "[static_map_proxy] target service returned empty map; "
                    "waiting for topic map on %s",
                    map_topic,
                )
            except rospy.ServiceException as exc:
                rospy.logerr_throttle(
                    2.0,
                    "[static_map_proxy] target call failed (%s -> %s): %s",
                    service_name,
                    target_service,
                    exc,
                )

        raise rospy.ServiceException(
            "No valid map available yet from target service or %s" % map_topic
        )

    rospy.Service(service_name, GetMap, _handle)
    if use_target_service:
        rospy.loginfo(
            "[static_map_proxy] serving %s from topic %s (fallback: %s)",
            service_name,
            map_topic,
            target_service,
        )
    else:
        rospy.loginfo(
            "[static_map_proxy] serving %s from topic %s", service_name, map_topic
        )
    rospy.spin()


if __name__ == "__main__":
    main()
