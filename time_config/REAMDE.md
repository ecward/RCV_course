# System configuration for time synchronization using a gps device

In order to synchronize the timestamps of ros-messages between
different devices we will use NTP with the chrony implementation.

There is one server (the NUC) that gets accurate time from a BU-353
gps device. Clients (other ros computers) connect as clients using
chrony to syncronize their clocks.

Importantly the velodynes support getting time information from a GPS
device so that the data we get from them will be synchronized with the
system clock.
           _______________
BU-353 -> | Chrony server | -> client
           ---------------
                   _______________
Garmin 18x lvc -> | Velodyne puck | -> client
                   ---------------

Since both GPS devices uses the time they get from the sattelites the
timestamps will be synchronized.

For other sensors more work is required since they do not directly keep
track of the "global time". Instead we will do a best effort approach
to time-stamp recived data using tuned time-delays.

It seems like you need PPS (pulse-per-second) signal to use chrony with
a gps and tbe BU-353 does not have this: http://catb.org/gpsd/hardware.html

The trimble might work.


## Server installation

sudo apt-get install gpsd
sudo apt-get install chrony

sudo apt-get install gpsd-clients //for debuging

## Server set-up

Plug in the BU-353 to USB port 0 (the "SS" one). To check that it's working run (as root):

    stty -F /dev/ttyUSB0 ispeed 4800 && cat < /dev/ttyUSB0
 
You might have to run the command twice.

The chrony server will listen to gps data that are provided on TCP port 2947
with gpsd. To check that gpsd is working:

Start (as a deamon) with:

    gpsd /dev/ttyUSB0 -n

Or with more debug stuff:

    gpsd /dev/ttyUSB0 -n -N -D3 -F /var/run/gpsd.sock

Then telnet to port 2947

    telnet localhost 2947

and see that we are getting data:

    ?WATCH={"enable":true,"json":true}

to stop getting data:

    ?WATCH={"enable":false}


## Server configuration

We will enable gpsd in a startup script

    TODO!

The chrony server

We need to allow computers on the same network to use the NUC as their NTP 
server in chrony.conf

   allow 130.237.218

Obvoiusly change to use the address of the current subnet.

    local startum 1

Which indicates that the NUC has access to a real time clock.

### Server debug

The password is listed in /etc/chrony/chrony.keys

> chronyc
> password vJgD1yif 
> clients
> settime 17:17:17

Settime should set the system clock and also the clients clock, note that
there are protections against bad estimates built in to chrony. 

So you might have to remove 

    maxupdateskew

To actually change the time

### Router configuration

In order for clients to connect to the NUC to get the time, we need a static
IP for the NUC.

## Client configuration

    sudo apt-get install chrony

Add the the ip of the NUC as as server

e.g.

130.237.218.75

service chrony restart


#TODOs

It wuld be nice to have a DNS server running on the nuc so that we 
can refer to laptops using that. But not really needed.