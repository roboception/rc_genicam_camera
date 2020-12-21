1.2.3 (2020-12-21)
------------------

* Updated cmake and CI files
* Dropped building for trusty and added building for focal on CI

1.2.2 (2019-09-06)
------------------

* Added parameter to rotate grabbed images by 180 degrees

1.2.1 (2019-06-06)
------------------

* Added parameter for storing all grabbed images as files
* Minor performance optimization

1.2.0 (2019-05-24)
------------------

* Fixed failing of loading initial GenICam parameters
* Fixed readme.
* Generate documentation too
* Report errors if config or calibration file cannot be loaded

1.1.0 (2019-02-01)
------------------

* Setting CameraInfo values properly from mono- or stereocamera calibration file
* Read camera timestamp from TimestampLatchValue by default, which in GenICam standard
* Mapping of most 8 and 16 bit pixel formats from GenICam to ROS

1.0.0 (2019-01-24)
------------------

* Initial release
