# MRSD_project

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
