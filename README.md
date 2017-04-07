# MRSD_project

Create catkin workspace and include everything in this repository in the src folder


------------------------------------------------------

*To launch the image capturing module:
  
  roslaunch flea3 stereo_node.launch 

*To launch the detection/classification module:
  
  roslaunch cv_tracker ssd.launch
    
    -To view detection information on person objects:

        rostopic echo /obj_person/image_obj

    -To view detection information on car objects:

        rostopic echo /obj_car/image_obj

*To launch the depth measure module: 
        
  roslaunch depth_measure depth_measure.launch


------------------------------------------------------

NOTE:

Please ONLY push the working code back to Github 

Depth information published by depth_measure (or elas_ros) does not seem correct...

I think the "depth measure" module can be used for sensor fusion task. So no need to have another seperate node for sensor fusion? 

Let's plan to use Rviz for visualization for now 
