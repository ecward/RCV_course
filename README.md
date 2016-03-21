# RCV_course

# Dependencies

    sudo apt-get install ros-indigo-people-msgs
    sudo apt-get install libpcap-dev

# Velodyne data

To play back pcap log

    roslaunch velodyne_pointcloud VLP16_points.launch pcap:=<path/to/pcap> device_ip=<velodyne_ip>

(Velodynes configures so that one is on 192.168.1.10 and the other 192.168.1.11)