
rc_genicam_camera
-----------------

This nodelet permits access to the configuration of a GenICam compatible camera
and publishes raw images and camera_info messages according to the ROS image
pipeline.

Configuration
-------------

#### Parameters

Parameters to be set to the ROS param server before run-time.

* `device`: The ID of the GenICam camera. It depends on the camera and the
  GenTL producer what the ID is. Available devices can be listed with
  'gc_config -l' from the rc_genicam_api package.

  See https://github.com/roboception/rc_genicam_api#device-id for more details.

* `gev_access`: The gev_access mode, i.e.:
  * 'control'   Configuration and streaming with the possibility of other
                clients to read GenICam parameters. This is the default.
  * 'exclusive' Exclusive access to the sensor. This prevents other clients to
                read GenICam parameters.

* `config_file`: Optional parameter with absolute path and name of a file that
  contains a one or more GenICam parameters or commands, separated by white
  spaces (e.g. space, return):

  <name>=<value> sets the given value to the parameter with the given name. It
                 must not contain a white space.

  <name>         Calls the GenICam command with the given name. The name must
                 not contain any white space.

  # ... \n       Comment which is skipped.

  Processing stops on the first error, which is reported in the logfile.

* `calib_file`:  Optional parameter with absolute path and name of a file that
   contains the camera calibration. The information is used for the CameraInfo
   messages:

   # Calibration matrix

   camera.A=[<fx> <skew> <u0>; 0 <fy> <v0>; 0 0 1]
   camera.width=<w>
   camera.height=<h>

   # Optional parameters for radial and tangential distortion. Missing
   # parameters are assumed to be 0

   camera.k1=<k1>
   camera.k2=<k2>
   camera.k3=<k3>
   camera.p1=<p1>
   camera.p2=<p2>

   # Alternative optional parameters for equidistant distortion model

   camera.e1=<e1>
   camera.e2=<e2>
   camera.e3=<e3>
   camera.e4=<e4>

Provided Topics
---------------

The following topics are provided:

* camera_info (sensor_msgs::CameraInfo)
* image_raw (sensor_msgs::Image)

Services
--------

The following service offers the possibility to request the value of a GenICam
parameter:

* `get_genicam_parameter` (rc_genicam_camera::GetGenICamParameter)

The following service offers the possibility to set GenICam parameters:

* `set_genicam_parameter` (rc_genicam_camera::SetGenICamParameter)

Launching
---------

* Using command line parameters:

      rosrun rc_genicam_camera rc_genicam_camera _device:=<ID>

* As a nodelet, and in a separate **namespace**:

      ROS_NAMESPACE=my_camera rosrun nodelet nodelet standalone rc_genicam_driver _device:=<ID>

