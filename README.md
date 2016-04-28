# get_out_of_collision

Provides a service to get out of a current collision state.

Works by searching a very close joint configuration that is out of collision and sending a goal there using MoveIt!'s `/execute_kinematic_path` service.

It's based on [Ioan Sucan](https://github.com/isucan) [fix_start_collision_state.cpp](https://github.com/ros-planning/moveit_ros/blob/indigo-devel/planning/planning_request_adapter_plugins/src/fix_start_state_collision.cpp).

Thanks to [Dave Coleman](https://github.com/davetcoleman) for the support too.

**Use with care.**

# Usage
Launch the node:

    roslaunch get_out_of_collision get_out_of_collision_node.launch

Its default parameters are as found in the launchfile:

````
<launch>
    <node name="get_out_of_collision_node" pkg="get_out_of_collision" type="get_out_of_collision_node" output="screen">
        <!-- Time for the trajectory to the out-of-collision waypoint -->
        <param name="time_for_goal" value="2.0" />
        <!-- Fraction of the maximum joint range to jiggle (range 0.0-1.0)-->
        <param name="jiggle_fraction" value="0.02" />
        <!-- Max sampling attempts -->
        <param name="max_sampling_attempts" value="100" />
        <!-- MoveIt! group name to use (the more joints the better) -->
        <param name="group_name" value="arm_torso" />
    </node>
</launch>
````


To trigger the behaviour just call the `/get_out_of_collision` service of type `std_srvs/Empty`:

    rosservice call /get_out_of_collision "{}"
