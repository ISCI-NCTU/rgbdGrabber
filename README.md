This project provides classes to grab RGBD frames from various sensors.

### Dependencies

This code depends on the following other libraries and was tested under Ubuntu
14.04. 
- Opencv 2 (2.3.1)
- [librealsense](https://github.com/IntelRealSense/librealsense)

### Install

This package uses [the pods build
system](http://sourceforge.net/p/pods/home/Home/). Used widely at CSAIL
MIT the build system makes it easy to break up software projects into
small packages that can be checked out and compiled automatically (see
below).

- *Linux:* 

    Clone this repository and compile the code:

    ```
    git clone git@github.com:jstraub/rgbdGrabber; cd rgbdGrabber;
    make checkout; make configure; make -j6; make install;
    ```
    
    Note that this will checkout several other necessary repositories.
    To update all repositories run
    
    ```
    make update; make configure; make -j6; make install;
    ```

