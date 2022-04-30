# attention-l4ros2

In this task, the robot will focus its attention to the specified objects in perimeter of 1 meter. In order to do it, a knowledge base will be used, specifically the [ros2_knowledge_graph](https://github.com/fmrico/ros2_knowledge_graph)

### List of objects to track:
- Person
- TODO

## Launch our edited world

    ros2 launch attention_l4ros2 attention_hospital_launch.py

## How to detect objects within a range

As we need to accomplish the task before the term ends, we'll be using the gazebo data to compare distances between models. We can do so by using the service given by gazebo in our edited world.

    ros2 service call /gazebo/get_entity_state gazebo_msgs/srv/GetEntityState '{name: 'tiago', reference_frame: 'world'}'

The output should be something like:
    
    requester: making request: gazebo_msgs.srv.GetEntityState_Request(name='tiago', reference_frame='world')

    response:
    gazebo_msgs.srv.GetEntityState_Response(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=750, nanosec=680000000), frame_id='world'), state=gazebo_msgs.msg.EntityState(name='', pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=-0.0007741418669663402, y=10.005518173651984, z=-0.00031884616825245127), orientation=geometry_msgs.msg.Quaternion(x=0.00020279364172472853, y=-0.0002460842640248263, z=-0.7119967294127421, w=0.7021827081485505)), twist=geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=1.1000766920860107e-05, y=-1.8712834094210468e-05, z=-0.0008444918926725874), angular=geometry_msgs.msg.Vector3(x=-7.014788892002443e-05, y=-0.0002989604435984749, z=-3.403399565462915e-05)), reference_frame=''), success=True)

## Using the knowledge graph
TODO
