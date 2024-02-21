# Differential drive

This is the `differential_drive` ROS2 version of the package written by Jon Stephan.
The originally published version of the package was released on
Google Code. When Google Code was archived, a copy was pulled into
GitHub.

The original package was written for ROS Groovy. In 2015 Mark Rose
made minor source changes to update the package for ROS Hydro and above,
moving to catkin, removing code warnings,
and changing the package layout to match current ROS guidelines.

2020 Andreas Klintberg update all of it to ROS2 and changed the layout to fit the new Python ROS2 guidelines.


ROS package page::
http://wiki.ros.org/differential_drive

Jon Stephan&rsquo;s repository location::
https://github.com/jfstepha/differential-drive

Original, archived code repository::
https://code.google.com/p/differential-drive/

## (Optional) Hardware and firmware

Included in this repository is now also a firmware to be used using a simple arduino and rotary encoders that are easily found for cheap on eBay.
The reason for this is because it's a super easy way to get two wheel rotary encoders data connected using only USB.

### Hardware and setup

Arduino UNO or similar.
Rotary encoder (assumes optical incremental encoder)

1.  Hook up the red power line to +5V and the black wire to ground.

2. Wire rotary encoder for the right (from the front) wheel the A and B lines (green and white) to Arduino D2 ad D3.

3. Wire rotary encoder for the left (from the front) wheel the A and B lines (green and white) to Arduino D4 ad D5.

Connect Arduino and upload the sketch found in the `firmware`-folder.

Open up the serial monitor and either turn each wheel or just rotate both rotary encoder axles, and make sure you get some reasonable output. 

## Installation
`colcon build --symlink-install`

Make sure to activate your workspace afterwards.

## Usage
`ros2 run differential_drive diff_tf`

`ros2 launch differential_drive differential_drive.launch.py`

## API Docs

Please refer to the ROS Wiki page for this package for API information:
http://wiki.ros.org/differential_drive.

## License
Unfortunately since the original repo that this was based on is GPL, this one also needs to be GPL. I asked the author to change the license
if that ever happens this will immediately follow that and become MIT. 

The firmware is taken from an MIT package.

The code is licensed under the terms of the Gnu General Public
License, Version 3. See the file `LICENSE` for details.


