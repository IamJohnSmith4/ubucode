<launch>

  <!-- TF: base_footprint → laser_link -->
  <node pkg="tf" type="static_transform_publisher" name="base_to_laser"
        args="0 0 0.2 0 0 0 -1.5708 base_footprint laser_link 50"/>

  <!-- map_server -->
  <node pkg="map_server" type="map_server" name="map_server"
        args="/home/ubutu2004/map/final.yaml"
        output="screen"/>

  <!-- AMCL -->
  <node pkg="amcl" type="amcl" name="amcl" output="screen"
        respawn="true">
    <param name="use_map_topic"       value="false"/>
    <param name="tf_broadcast"        value="true"/>
    <param name="odom_model_type"     value="diff"/>
    <param name="odom_frame_id"       value="odom"/>
    <param name="base_frame_id"       value="base_footprint"/>
    <param name="global_frame_id"     value="map"/>
    <param name="laser_model_type"    value="likelihood_field"/>
    <param name="min_particles"       value="100"/>
    <param name="max_particles"       value="500"/>
    <param name="laser_max_range"     value="7.0"/>
    <param name="laser_max_beams"     value="60"/>
    <param name="update_min_d"        value="0.1"/>
    <param name="update_min_a"        value="0.2"/>
    <param name="transform_tolerance" value="1.0"/>
    <param name="initial_pose_x"      value="0.0"/>
    <param name="initial_pose_y"      value="0.0"/>
    <param name="initial_pose_a"      value="0.0"/>
    <param name="gui_publish_rate"    value="10.0"/>
  </node>

  <!-- move_base -->
  <node pkg="move_base" type="move_base" name="move_base" output="screen">
    <rosparam file="$(find my_robot)/config/costmap_common.yaml"
              command="load" ns="global_costmap"/>
    <rosparam file="$(find my_robot)/config/costmap_common.yaml"
              command="load" ns="local_costmap"/>
    <rosparam file="$(find my_robot)/config/global_costmap.yaml"
              command="load"/>
    <rosparam file="$(find my_robot)/config/local_costmap.yaml"
              command="load"/>
    <rosparam file="$(find my_robot)/config/base_local_planner.yaml"
              command="load"/>
    <param name="base_global_planner"
           value="navfn/NavfnROS"/>
    <param name="base_local_planner"
           value="base_local_planner/TrajectoryPlannerROS"/>
    <remap from="cmd_vel" to="/mobile_base/commands/velocity"/>
  </node>


</launch>
