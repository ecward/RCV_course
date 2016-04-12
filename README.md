# RCV_course

# Dependencies

    sudo apt-get install ros-indigo-people-msgs
    sudo apt-get install libpcap-dev

# Velodyne data

To play back pcap log

    roslaunch velodyne_pointcloud VLP16_points.launch pcap:=<path/to/pcap> device_ip=<velodyne_ip>

(Velodynes configures so that one is on 192.168.1.10 and the other 192.168.1.11)

## Get roadmap for rviz

https://api.tiles.mapbox.com/v4/mapbox.streets/{z}/{x}/{y}.png?access_token=pk.eyJ1IjoiZXdhcmQiLCJhIjoiY2ltd2IycHV0MDBiOXZ3bTE5ZHBpZGNvOCJ9.6Cn4AnAZIKEkVVRPQpDgwg