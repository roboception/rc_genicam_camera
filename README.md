
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

#### Parameters for Synchronization

The following parameters are used for receiving and associating the time stamp
from the camera info of a hardware synchronized master camera that is
synchronized to the system clock.

* `host_timestamp`: True for using the host time stamp instead of the camera
  timestamp as image acquisition timestamp. This requires the availability of
  the command "TimestampLatch" and the parameter "Timestamp" in the nodemap of
  the camera.

  Default: false

* `timestamp_tolerance`: Maximum acceptable tolerance in seconds for
  determining the offset between the system and camera clock. No images are
  delivered if the offset cannot be determined with this tolerance.

  Default: 0.01

* `sync_info`: Name of topic that provides CameraInfo messages at the time
  when image acquisition is triggered via a hardware signal, e.g. CameraInfo
  of a camera that is hardware synchronized. The timestamps of these CameraInfo
  messages are used as image acquisition time.

  Default: (do not synchronize timestamps)

* `sync_tolerance`: Maximum acceptable tolerance in seconds between the
  CameraInfo timestamps and the image acquisition timestamps (i.e. either
  camera or host timestamp if host_timestamp is true). This must be set to less
  than half of the image period.

  Default: 0.019

Subscribed Topics
-----------------

If `reset_info` AND `sync_info` parameters are defined, then their values are
used to subscribe to these topics for determining the image timestamp.

* <reset_info> (sensor_msgs::CameraInfo)
* <sync_info> (sensor_msgs::CameraInfo)

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

