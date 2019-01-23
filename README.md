# wp3_calibrator
ROS package for automatic extrinsic pose calibration of multiple RGB-D sensor nodes. As described in the referenced paper, the use of large retroreflective markers is recommended for large volumes. 

**Keywords:** ROS, RGB-D, calibration

### License
The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Atle Aalerud<br />
Affiliation: [SFI Offshore Mechatronics](https://sfi.mechatronics.no/), [University of Agder](https://www.uia.no/en)<br />
Maintainer: Atle Aalerud, atle.aalerud@uia.no**

The wp3_calibrator package has been tested under [ROS] Indigo and Ubuntu 16.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

![Example image](doc/example.png)

### Publications

If you use this work in an academic context, please cite the following publication(s):

* A. Aalerud, J. Dybedal, and G. Hovland, **Automatic Calibration of an Industrial RGB-D Camera Network using Retroreflective Fiducial Markers**. Submitted. to Sensors, 2018. (In review)

        @article{Aalerud2018c,
            author = {Aalerud, Atle and Dybedal, Joacim and Hovland, Geir},
            journal = {submitted to Sensors},
            title = {{Automatic Calibration of an Industrial RGB-D Camera Network using Retroreflective Fiducial Markers}},
            year = {2018}
        }

* A. Aalerud, J. Dybedal, E. Ujkani, and G. Hovland, **Industrial Environment Mapping Using Distributed Static 3D Sensor Nodes**, in 2018 14th IEEE/ASME International Conference on Mechatronic and Embedded Systems and Applications (MESA), 2018. ([PDF](https://ieeexplore.ieee.org/document/8449203/))

        @inproceedings{Aalerud2018a,
            author = {Aalerud, Atle and Dybedal, Joacim and Ujkani, Erind and Hovland, Geir},
            booktitle = {2018 14th IEEE/ASME Int. Conf. Mechatron. Embed. Syst. Appl.},
            month = {jul},
            publisher = {IEEE},
            title = {{Industrial Environment Mapping Using Distributed Static 3D Sensor Nodes}},
            year = {2018}
        }


## Installation
### Building from Source
#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Eigen] (linear algebra library)
    ````bash
    sudo apt-get install libeigen3-dev
    ````

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using
````bash
cd catkin_workspace/src
git clone https://github.com/SFI-Mechatronics/wp3_calibrator.git
cd ../
catkin_make
````

## Usage

To run the calibration on logged data from the Industrial Robotics Lab at UiA:
* Download the ROS bag from (to be added)
* Decompress
    ````bash
    rosbag decompress *.bag
    ````	
* Edit path to the ROS bag in calibrate.launch
* Run calibration using:
    ````bash
    roslaunch wp3_calibrator calibrate.launch
    ````
        
The final calibration of the journal paper may be applied directly by using:
````bash
roslaunch wp3_calibrator tf_resultC.launch
````

To calibrate your own sensors you need to edit or copy calibrate.launch with following updates:
* Disable playback of rosbag.
* Set num_sensors to number of sensors to calibrate
* Update list of topics according to your sensors

## Config files

* **include/wp3_calibrator/defines.h** Program configurations (reguires recompile)
    * Dimensions of aruco markers
    * Padding size around aruco markers
    * Calibration parameters

## Launch files

* **calibrate.launch:** Sets parameters and starts calibration
* **tf_resultC.launch** Launch static transform publishers for final calibration results from journal.


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/SFI-Mechatronics/wp3_calibrator/issues).


[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[Eigen]: http://eigen.tuxfamily.org
