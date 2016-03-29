#!/usr/bin/env python

import rospy
import numpy as np
#from rosgraph_msgs.msg import Clock
from velodyne_msgs.msg import ExternalTimeSource
import tf
import scipy.io as sio
import datetime
import time
from LatLongUTMconversion import *

'''
Read trimble data from .mat files and publish 
current pose (tf from map to trimble) and time
emulating real-time

'''

import scipy.io as spio

def loadmat(filename):
    '''
    this function should be called instead of direct spio.loadmat
    as it cures the problem of not properly recovering python dictionaries
    from mat files. It calls the function check keys to cure all entries
    which are still mat-objects
    '''
    data = spio.loadmat(filename, struct_as_record=False, squeeze_me=True)
    return _check_keys(data)

def _check_keys(dict):
    '''
    checks if entries in dictionary are mat-objects. If yes
    todict is called to change them to nested dictionaries
    '''
    for key in dict:
        if isinstance(dict[key], spio.matlab.mio5_params.mat_struct):
            dict[key] = _todict(dict[key])
    return dict        

def _todict(matobj):
    '''
    A recursive function which constructs from matobjects nested dictionaries
    '''
    dict = {}
    for strg in matobj._fieldnames:
        elem = matobj.__dict__[strg]
        if isinstance(elem, spio.matlab.mio5_params.mat_struct):
            dict[strg] = _todict(elem)
        else:
            dict[strg] = elem
    return dict


gps_fields = {'lat':0,'lat_ts':1,'lon':2,'lon_ts':3,'altitude':4,'heading':6}



class TrimblePlayer:
    
    def __init__(self,utm_data,time_data):
        '''
        utm_data  [[e,n,heading],...]
        '''
        
        self.utm_data = utm_data
        self.time_data = time_data
        
        self.clock_pub = rospy.Publisher('playback_clock',ExternalTimeSource,queue_size=10)
        self.tf_pub    = tf.TransformBroadcaster()
        
        rospy.init_node('trimble_player')
        #rospy.set_param('/use_sim_time', True)

        #We have problems with the timestamps,
        #it should be almost exactly 10 ms between the ticks, but sometimes it is not,

        diff_t = np.diff(self.time_data)

        #Let's just make it easy for ourselves. Take times such that
        #time is monotonically increasing
        OK_time_idx = [0]
        for i in range(1,np.size(self.time_data)):
            if self.time_data[i] > self.time_data[OK_time_idx[-1]]:
                OK_time_idx.append(i)

        last_print_time = 0
        for i in range(0,len(OK_time_idx)-1):
            t_idx = OK_time_idx[i]
            if not rospy.is_shutdown():
                t0 = time.time()
                
                t = self.time_data[t_idx]
                pose = self.utm_data[t_idx]

                #c = Clock()            
                #c.clock.secs = int(t)
                #c.clock.nsecs = int((t-int(t))*1000000000)
                c = ExternalTimeSource()
                c.header.stamp = rospy.Time.now()
                c.unix_time_s  = t
                self.clock_pub.publish(c)

                #publish transform between "map" and "trimble"
                self.tf_pub.sendTransform((pose[0],pose[1],0.0), 
                                          tf.transformations.quaternion_from_euler(0,0,pose[2]),
                                          c.header.stamp,
                                          "trimble",
                                          "map")

                
                if (t - last_print_time) > 0.995:
                    print "Clock-time = ",datetime.datetime.fromtimestamp(t).strftime("%d/%m/%Y %H:%M:%S.%f")
                    print "unix_time = ",t
                    last_print_time = t


                comp_time = time.time()-t0                
                sleep_time = self.time_data[OK_time_idx[i+1]]-t-comp_time
                # print "sleep_time = ",sleep_time
                if sleep_time < 0:
                    sleep_time = 0
                    print "WARNING SLEEP TIME < 0!"            
                time.sleep(sleep_time)
            
    

if __name__ == '__main__':
    trimble_data = loadmat('/media/erik/3tb/RCV_data/arlanda_test_track_wireshark_18032016/Loop1.mat')
    #trimble_data = loadmat('/media/erik/3tb/RCV_data/arlanda_test_track_wireshark_18032016/Loop2.mat') #this is the same data as Loop1... fuck
    #trimble_data = loadmat('/media/erik/3tb/RCV_data/arlanda_test_track_wireshark_18032016/ThirdRunBehindTruckStephanos30and50.mat')

    data =  trimble_data["Results"]["GPS"]["data"]
    lat = data[:,gps_fields['lat']]; lat_ts = data[:,gps_fields['lat_ts']]
    lon = data[:,gps_fields['lon']]; lon_ts = data[:,gps_fields['lon_ts']]
    heading = data[:,gps_fields['heading']]
    lat_ok = lat>0
    lat = lat[lat_ok]; lat_ts = lat_ts[lat_ok]
    lon = lon[lat_ok]; lon_ts = lon_ts[lat_ok]
    #heading from north in degrees (clockwise...)
    heading = heading[lat_ok]
    heading = np.pi/180*heading
    heading = np.pi/2-heading

    N = len(lat)
    utm = np.zeros((N,3)) #assume all the same zone
    for i in range(0,N):
        (z,e,n) = LLtoUTM(23,lat[i],lon[i])
        utm[i,0] = e # "x"
        utm[i,1] = n # "y"
        utm[i,2] = heading[i]

    #Convert time to unixtime (seconds) instead of milliseconds since 2004-01-01
    s = "01/01/2004"
    t0 = time.mktime(datetime.datetime.strptime(s,"%d/%m/%Y").timetuple())
    t_unix = t0 + lat_ts/1.0e3

    #Ignore some crap in the beginning (say first 100)
    ok_idx = 100
    
    TrimblePlayer(utm[ok_idx:,:],t_unix[ok_idx:])
