#!/usr/bin/env python

import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *


def processFeedback(feedback):
    p = feedback.pose.position
    print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)

if __name__=="__main__":
    rospy.init_node("bottle")

    server = InteractiveMarkerServer("bottle")

    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "table_top"
    int_marker.name = "bottle"
    int_marker.description = "Simple Bottle"

    bottle = Marker()
    bottle.type = Marker.CYLINDER
    bottle.scale.x = 0.076
    bottle.scale.y = 0.076
    bottle.scale.z = 0.15
    bottle.color.r = 0.9
    bottle.color.g = 0.05
    bottle.color.b = 0.1
    bottle.color.a = 0.75

    bottle.pose.position.z = 0.075

    bottle_control = InteractiveMarkerControl()
    bottle_control.always_visible = True
    bottle_control.markers.append( bottle )

    int_marker.controls.append( bottle_control )

    rotate_control = InteractiveMarkerControl()
    rotate_control.name = "move_x"
    rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

    int_marker.controls.append( rotate_control )

    server.insert(int_marker)

    server.applyChanges()

    rospy.spin()
