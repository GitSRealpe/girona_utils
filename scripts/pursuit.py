import rospy
import numpy as np
from nav_msgs.msg import Path, Odometry

from visualization_msgs.msg import Marker, MarkerArray

sphere_marker = Marker()
sphere_marker.type = Marker.SPHERE
sphere_marker.id = 0
sphere_marker.color.a = 0.7
sphere_marker.color.r = 1
sphere_marker.color.g = 1
sphere_marker.color.b = 1
sphere_marker.scale.x = 0.5
sphere_marker.scale.y = 0.5
sphere_marker.scale.z = 0.5
# poner esto en el frame del girona asi no tener que hacer update con la odometria?
sphere_marker.header.frame_id = "world_ned"

intersection_marker1 = Marker()
intersection_marker1.type = Marker.SPHERE
intersection_marker1.header.frame_id = "world_ned"
intersection_marker1.id = 1
intersection_marker1.color.a = 1
intersection_marker1.color.r = 1
intersection_marker1.color.g = 1
intersection_marker1.color.b = 0
intersection_marker1.scale.x = 0.1
intersection_marker1.scale.y = 0.1
intersection_marker1.scale.z = 0.1
intersection_marker1.pose.orientation.w = 1
# poner esto en el frame del girona asi no tener que hacer update con la odometria?
intersection_marker2 = Marker()
intersection_marker2.type = Marker.SPHERE
intersection_marker2.header.frame_id = "world_ned"
intersection_marker2.id = 2
intersection_marker2.color.a = 1
intersection_marker2.color.r = 1
intersection_marker2.color.g = 1
intersection_marker2.color.b = 0
intersection_marker2.scale.x = 0.1
intersection_marker2.scale.y = 0.1
intersection_marker2.scale.z = 0.1
intersection_marker2.pose.orientation.w = 1


def callback(data: Odometry):
    sphere_marker.pose = data.pose.pose

    intersections = sphere_line_intersection(
        (
            data.pose.pose.position.x,
            data.pose.pose.position.y,
            data.pose.pose.position.z,
        ),
        0.25,
        (
            path.poses[0].pose.position.x,
            path.poses[0].pose.position.y,
            path.poses[0].pose.position.z,
        ),
        (
            path.poses[1].pose.position.x,
            path.poses[1].pose.position.y,
            path.poses[1].pose.position.z,
        ),
    )
    print(intersections)

    intersection_marker1.pose.position.x = intersections[0][0]
    intersection_marker1.pose.position.y = intersections[0][1]
    intersection_marker1.pose.position.z = intersections[0][2]


# print("got odom")


rospy.init_node("pursuit_controller")

path: Path = rospy.wait_for_message("/iauv_planner/path", Path)
print("path gotten")

rospy.Subscriber("/girona1000/navigator/odometry", Odometry, callback)

pub = rospy.Publisher("pursuit_viz", MarkerArray, queue_size=10)


def sphere_line_intersection(sphere_center, sphere_radius, line_start, line_end):
    # Unpack sphere parameters
    x_c, y_c, z_c = sphere_center
    r = sphere_radius
    # Unpack line parameters
    x_0, y_0, z_0 = line_start
    x_1, y_1, z_1 = line_end
    # Coefficients of the quadratic equation C + B*t + A*t^2 = 0
    a = (x_0 - x_1) ** 2 + (y_0 - y_1) ** 2 + (z_0 - z_1) ** 2
    c = (x_0 - x_c) ** 2 + (y_0 - y_c) ** 2 + (z_0 - z_c) ** 2 - r**2
    b = (x_1 - x_c) ** 2 + (y_1 - y_c) ** 2 + (z_1 - z_c) ** 2 - c - a - r**2
    # Calculate the discriminant
    discriminant = b**2 - (4 * a * c)

    if discriminant < 0:
        # No intersection
        return None
    elif discriminant == 0:
        # One intersection (tangent line)
        t = -b / (2 * a)
        intersection = np.array([x_0, y_0, z_0]) + t * np.array([x_1, y_1, z_1])
        return [intersection]
    else:
        # Two intersections
        t1 = (-b + np.sqrt(discriminant)) / (2 * a)
        t2 = (-b - np.sqrt(discriminant)) / (2 * a)
        intersection1 = (
            np.array([x_0, y_0, z_0]) * (1 - t1) + np.array([x_1, y_1, z_1]) * t1
        )
        intersection2 = (
            np.array([x_0, y_0, z_0]) * (1 - t2) + np.array([x_1, y_1, z_1]) * t2
        )
        return [intersection1, intersection2]


# Example usage:
sphere_center = (0, 0, 0)  # Center of the sphere
sphere_radius = 0.1  # Radius of the sphere
line_start = (-10, -10, -10)
line_end = (10, 10, 10)
# line_start = (-7.0, -1.75, 0.0)
# line_end = (-6.0, 2.0, 0.0)
line_start = (1.88, -6.37, 0.0)
line_end = (-4.35, 5.42, 3.19)

intersections = sphere_line_intersection(
    sphere_center, sphere_radius, line_start, line_end
)
print(intersections)


marker_array = MarkerArray()
marker_array.markers.append(sphere_marker)
marker_array.markers.append(intersection_marker1)
marker_array.markers.append(intersection_marker2)


while not rospy.is_shutdown():

    pub.publish(marker_array)
    rospy.Rate(10).sleep()


# rospy.spin()
