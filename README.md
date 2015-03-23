# asctec_sdk3_framework
ROS metapackage to interface with Asctec UAVs running latest firmware update, which requires using SDK v3.0

This metapackage has been implemented from the ground up to reproduce (as much as possible) the same characteristics of the ubiquitous http://wiki.ros.org/asctec_mav_framework.

The main differences are:
  - It uses the latest SDK v3.0 and the ACI communication API from Asctec 
  - It requires flashing a modified version (included) of the latest firmware
  - There is no Kalman filter implemented in this firmware version (TODO)
  - GPS waypoints are accepted, provided a geofence if properly initialised
  
Documentation is notably null at the moment, but the code has been tested with ROS Hydro and Indigo using the Asctec Pelican quadrotor.

This metapackage also includes a ROS package containing the tutorials provided by Asctec for the ACI, as well as the ACI remote v100.

Those packages, along with the firmware, are distributed under GPLv3 license, whilst the other ROS packages are distributed under BSD license (as usual with ROS packages).
