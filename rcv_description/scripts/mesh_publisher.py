#!/usr/bin/env python
# license removed for brevity
import rospy
import roslib
from visualization_msgs.msg import Marker


def publish_mesh_msg():
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rospy.init_node('mesh_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        m = Marker()
        m.header.frame_id = "/base_link"
        m.ns = "meshes"
        m.id = 0
        m.action = Marker.ADD
        #m.lifetime = rospy.Duration(2)
        m.frame_locked = True

        m.type = Marker.MESH_RESOURCE
        m.mesh_resource = "package://rcv_description/meshes/RCV_assembly_stl_downsample.dae"
        m.scale.x = 0.025
        m.scale.y = 0.025
        m.scale.z = 0.025
        m.color.r = 220/255.0
        m.color.g = 220/255.0
        m.color.b = 220/255.0
        m.color.a = 0.8
        
       
        pub.publish(m)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_mesh_msg()
    except rospy.ROSInterruptException:
        pass


