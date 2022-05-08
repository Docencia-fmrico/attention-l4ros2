# attention-l4ros2

## Description

Repostory for paying attention to specific models on the [aws hospital world](https://github.com/aws-robotics/aws-robomaker-hospital-world#readme).

## How to run

To launch the world and spawn tiago:

    ros2 launch attention_l4ros2 attention_hospital_launch.py

To launch our attention nodes:

    ros2 launch attention_l4ros2 attentio_launch.py

## Plugin

To check the coordinates of the models in the world, we will make use of this plugin:

    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
        <ros>
            <namespace>/gazebo</namespace>
        </ros>
        <update_rate>1.0</update_rate>
    </plugin>

These \<plugin> tag need to be inside the \<world> tag.

This was used to make it easier and focus on the attention programming.

We can check that the plugin is correctly working by executing this command that makes use of the service that the plugin provides:

    ros2 service call /gazebo/get_entity_state gazebo_msgs/srv/GetEntityState '{name: 'BPCart_3::BPCart::body', reference_frame: 'tiago::base_footprint'}'

In this case we are getting the state of the model "BPCart_3" with respect to tiago.

## Models to pay attention

Tiago will be paying attention to people, here we can se some of these models:

![models to pay attention](./imgs/attention_models.png)

## Video demostration:

<TODO>

## Authors

 - Javier de la Canóniga: @javi-dbgr
 - Iván López: @ivrolan
 - Alejandro Moncalvillo: @Amglega
 - Unai Sanz: @USanz


