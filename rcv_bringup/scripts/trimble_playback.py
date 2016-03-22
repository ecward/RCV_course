#!/usr/bin/env python

import rospy
import numpy as np
from rosgraph_msgs.msg import Clock
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
        
        self.clock_pub = rospy.Publisher('/clock',Clock,queue_size=10)
        self.tf_pub    = tf.TransformBroadcaster()
        
        rospy.init_node('trimble_player')
        rospy.set_param('/use_sim_time', True)

        #We have problems with the timestamps,
        #it should be almost exactly 10 ms between the ticks, but sometimes it is not,

        diff_t = np.diff(self.time_data)

        #A datum is OK if it is 10ms to previous or to next
        #Algorithm is:
        # 1. find next_OK > prev_OK
        # 2. set prev_OK = next_OK
        # 3. find next_OK
        # 4. publish prev_OK
        # 5. Sleep until next_OK


        def diff_OK(diff):
            return diff > 0 and np.abs(diff-0.01) < 1.0e-5
        
                
        def find_nextOK(prev_OK):
            i = prev_OK+1
            while i < np.size(self.time_data)-1:
                diff_prev = self.time_data[i]-self.time_data[i-1]
                diff_next = self.time_data[i+1]-self.time_data[i]
                if diff_OK(diff_prev) or diff_OK(diff_next):
                    return i
                i += 1
            
            print "No more data"
            return -1

        prev_OK = 0
        last_print_time = 0
        
        #loop until we have output all the gps data
        for i in range(1,np.size(utm_data,0)):
            if not rospy.is_shutdown():
                t0 = time.time()
                next_OK = find_nextOK(prev_OK)
                if next_OK < 0:
                    break

                prev_OK = next_OK

                t = self.time_data[prev_OK]
                pose = self.utm_data[prev_OK]

                c = Clock()            
                c.clock.secs = int(t)
                c.clock.nsecs = int((t-int(t))*1000000000)
                self.clock_pub.publish(c)

                

                #publish transform between "map" and "trimble"
                self.tf_pub.sendTransform((pose[0],pose[1],0.0), 
                                          tf.transformations.quaternion_from_euler(0,0,pose[2]),
                                          c.clock,
                                          "trimble",
                                          "map")


                next_OK = find_nextOK(prev_OK)
                if next_OK < 0:
                    break

                if (t - last_print_time) > 0.995:
                    print "Clock-time = ",datetime.datetime.fromtimestamp(t).strftime("%d/%m/%Y %H:%M:%S.%f")
                    print "unix_time = ",t
                    last_print_time = t

                #rospy.spin_once()

                comp_time = time.time()-t0
                #Sleep until next timestamp, accounting for computation time                
                #print "comp_time = ",comp_time
                #print "delta_time = ",self.time_data[next_OK]-self.time_data[prev_OK]
                sleep_time = self.time_data[next_OK]-self.time_data[prev_OK]-comp_time
                # print "sleep_time = ",sleep_time
                if sleep_time < 0:
                    sleep_time = 0
                    print "WARNING SLEEP TIME < 0!"            
                time.sleep(sleep_time)
            
    

if __name__ == '__main__':
    trimble_data = loadmat('/media/erik/3tb/RCV_data/arlanda_test_track_wireshark_18032016/Loop1.mat')

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
