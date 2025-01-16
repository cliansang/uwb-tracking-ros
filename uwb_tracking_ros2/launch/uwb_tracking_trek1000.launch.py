#<launch>
#    <arg name="use_static_tf" default="true" />
#    <node pkg="tf" type="static_transform_publisher" name="uwb_static_tf"
#      args="2.5 2.5 0 3.14159 0 0 map uwb_map 30"
#      if="$(arg use_static_tf)" />
#
#    <!--Exctract info from dwm1001 dev boards, filter them and publish them into topics-->
#    <node pkg="uwb_tracking_ros" type="uwb_tracking_trek1000.py" name="uwb_tracking_trek1000" output="screen">
#        <rosparam file="$(find uwb_tracking_ros)/cfg/params_trek1000.yaml" command="load"/>
#    </node> 
#
#    <!-- Display topics of the tags into Rviz-->
#    <node pkg="uwb_tracking_ros" type="viz_trek1000.py" name="visualize_trek1000" />
#    <!-- Run rviz with the saved rviz file-->
#    <node type="rviz" name="rviz" pkg="rviz" args="-d $(find uwb_tracking_ros)/rviz/trek1000_rviz_config.rviz" />
# 
#</launch>
#