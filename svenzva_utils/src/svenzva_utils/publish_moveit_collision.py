#!/usr/bin/env python
import sys
import rospy
import shape_msgs.msg
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

def publish_scene_collision():
  rospy.init_node('publish_scene_collision', anonymous=True)
  collision_pub = rospy.Publisher('/collision_object', moveit_msgs.msg.CollisionObject, queue_size=1)
  coll_ob = moveit_msgs.msg.CollisionObject()
  shape = shape_msgs.msg.SolidPrimitive()

  #Create ground plane shape
  shape.type = shape.BOX
  shape.dimensions = [1.4, 1.4, 0.01]

  #Create ground plane pose
  g_pose = geometry_msgs.msg.Pose()
  g_pose.position.z = -0.01


  #Create buffer shapes
  buff1 = shape_msgs.msg.SolidPrimitive()
  buff1.type = buff1.BOX
  buff1.dimensions = [0.6, 1.4, 0.05]
  buff1_pose = geometry_msgs.msg.Pose()
  buff1_pose.position.x = -0.4

  buff2_pose = geometry_msgs.msg.Pose()
  buff2_pose.position.x = 0.4

  buff3 = shape_msgs.msg.SolidPrimitive()
  buff3.type = buff3.BOX
  buff3.dimensions = [1.4, 0.6, 0.05]
  buff3_pose = geometry_msgs.msg.Pose()
  buff3_pose.position.y = -0.4

  buff4_pose = geometry_msgs.msg.Pose()
  buff4_pose.position.y = 0.4

  #Create a ground plane
  coll_ob.operation = coll_ob.ADD
  coll_ob.header.frame_id = "base_link"
  coll_ob.id = "ground_plane"
  coll_ob.primitives.append(shape)
  coll_ob.primitive_poses.append(g_pose)

  coll_ob.primitives.append(buff1)
  coll_ob.primitive_poses.append(buff1_pose)

  coll_ob.primitives.append(buff1)
  coll_ob.primitive_poses.append(buff2_pose)

  coll_ob.primitives.append(buff3)
  coll_ob.primitive_poses.append(buff3_pose)

  coll_ob.primitives.append(buff3)
  coll_ob.primitive_poses.append(buff4_pose)


  #Annoyingly need to publish many times, if not spinning continuously
  collision_pub.publish(coll_ob)
  rospy.sleep(1.0)
  collision_pub.publish(coll_ob)
  collision_pub.publish(coll_ob)
  collision_pub.publish(coll_ob)
  rospy.sleep(1.0)

if __name__=='__main__':
  try:
    publish_scene_collision()
  except rospy.ROSInterruptException:
    pass
