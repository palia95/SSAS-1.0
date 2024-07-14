#!/usr/bin/env python
import RPi.GPIO as GPIO
import sys
import pprint
import requests
import time
import xmltodict
from geopy.distance import great_circle
from datetime import datetime
import json
import math
import pandas as pd
import paho.mqtt.subscribe as subscribe
from pandas import DataFrame

GPIO.setmode(GPIO.BCM) # Choose BCM to use GPIO numbers instead of pin numbers
GPIO.setwarnings(False)
interval = 6
pin1 = 4
pin2 = 17
pin3 = 27
pin4 = 22

#Define Antennas pins
GPIO.setup(pin1, GPIO.OUT)
GPIO.setup(pin2, GPIO.OUT)
GPIO.setup(pin3, GPIO.OUT)
GPIO.setup(pin4, GPIO.OUT)

#Support Variables
DELTA_GNSS = 0.05 #equivalent to about 5-6 km
DELTA_GNSS_INIT = 0.15 #equivalent to about 10-11 km
DELTA_POS = 0.0001 #equivalent to about 100 m
bts_latitude = 0
bts_longitude = 0
shiftdegree = 2 #degrees tollerated befoe switchin antenna

def InitBTSDB():
    #six decimal are enough for a decimeter precision
    #0.1 = 11.1 km
    #0.01 = 1.11 km
    #...
    #0.000001 = 0.111 m
    uav_latitude = 45.0059289
    uav_longitude = 10.461025
    uav_coo = (uav_latitude, uav_longitude)

    #Load BTS database
    BTS_DATA = pd.read_csv('bts_windtre.csv', sep=';')
    BTS_DATA = BTS_DATA.drop(columns=['tech','mcc','mnc','lac_tac','cid','psc_pci','arfcn','azimuth','height','tilt_mech','tilt_el'])
    BTS_DATA = BTS_DATA.round(4)

    BTS_DATA = BTS_DATA.drop_duplicates(subset=['site_name'])
    BTS_DATA = BTS_DATA.drop_duplicates(subset=['cell_lat'])
    BTS_DATA = BTS_DATA.drop_duplicates(subset=['cell_long'])

    ###In order to limit processing, limit the DF to cells in 10km radius
    Nearby_Cells_Lat = abs(BTS_DATA['cell_lat']-uav_coo[0])<=DELTA_GNSS_INIT
    Nearby_Cells_Lon = abs(BTS_DATA['cell_long']-uav_coo[1])<=DELTA_GNSS_INIT

    BTS_DATA = BTS_DATA[Nearby_Cells_Lon]
    BTS_DATA = BTS_DATA[Nearby_Cells_Lat]

    print("BTS DB Loaded")

    return(BTS_DATA)

#Bearing Angle
def calculate_bearing(pointA, pointB):
    """
    Calculates the bearing between two points.
    The formulae used is the following:
        θ = atan2(sin(Δlong).cos(lat2),
                  cos(lat1).sin(lat2) − sin(lat1).cos(lat2).cos(Δlong))
    :Parameters:
      - `pointA: The tuple representing the latitude/longitude for the
        first point. Latitude and longitude must be in decimal degrees
      - `pointB: The tuple representing the latitude/longitude for the
        second point. Latitude and longitude must be in decimal degrees
    :Returns:
      The bearing in degrees
    :Returns Type:
      float
    """
    if (type(pointA) != tuple) or (type(pointB) != tuple):
        raise TypeError("Only tuples are supported as arguments")

    lat1 = math.radians(pointA[0])
    lat2 = math.radians(pointB[0])

    diffLong = math.radians(pointB[1] - pointA[1])

    x = math.sin(diffLong) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1)
            * math.cos(lat2) * math.cos(diffLong))

    initial_bearing = math.atan2(x, y)

    # Now we have the initial bearing but math.atan2 return values
    # from -180° to + 180° which is not what we want for a compass bearing
    # The solution is to normalize the initial bearing as shown below
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360

    return compass_bearing

#Find nearby cells
def CellFind(uav_coo, BTS_DATA):
    global DELTA_GNSS

    Nearby_Cells_Lat = abs(BTS_DATA['cell_lat']-uav_coo[0])<=DELTA_GNSS
    Nearby_Cells_Lon = abs(BTS_DATA['cell_long']-uav_coo[1])<=DELTA_GNSS

    NEARBY_BTS = BTS_DATA[Nearby_Cells_Lon]
    NEARBY_BTS = NEARBY_BTS[Nearby_Cells_Lat]

    #Compute distance and bearing of nearby cells
    nearby_bts_distances = []
    nearby_bts_bearing = []

    for i in range(len(NEARBY_BTS)):
        cell = NEARBY_BTS.iloc[i]
        bts_coo = (cell.cell_lat, cell.cell_long)
        nearby_bts_distances.append(great_circle(uav_coo, bts_coo).m)
        nearby_bts_bearing.append(calculate_bearing(uav_coo, bts_coo))


    NEARBY_BTS['distance'] = nearby_bts_distances
    NEARBY_BTS['bearing'] = nearby_bts_bearing

    NEARBY_BTS = NEARBY_BTS.sort_values(by='distance')

    return(NEARBY_BTS.iloc[0])

#Turn ON Antenna 1
def Ant1(): 
    print("Turning ON Ant1")   
    GPIO.output(pin1, 0)
    GPIO.output(pin2, 0)
    GPIO.output(pin3, 0)
    GPIO.output(pin4, 0)

#Turn ON Antenna 2
def Ant2(): 
    print("Turning ON Ant2")   
    GPIO.output(pin1, 0)
    GPIO.output(pin2, 1)
    GPIO.output(pin3, 0)
    GPIO.output(pin4, 1)

#Turn ON Antenna 3
def Ant3():    
    print("Turning ON Ant3")
    GPIO.output(pin1, 1)
    GPIO.output(pin2, 0)
    GPIO.output(pin3, 1)
    GPIO.output(pin4, 0)

#Turn ON Antenna 4
def Ant4(): 
    print("Turning ON Ant4")   
    GPIO.output(pin1, 1)
    GPIO.output(pin2, 1)
    GPIO.output(pin3, 1)
    GPIO.output(pin4, 1)

#Huawei API Init
class HuaweiE3372(object):
  BASE_URL = 'http://{host}'
  COOKIE_URL = '/html/index.html'
  XML_APIS = [
    '/api/device/information',
    '/api/device/signal',
  ]
  session = None

  def __init__(self,host='192.168.8.1'):
    self.host = host
    self.base_url = self.BASE_URL.format(host=host)
    self.session = requests.Session()
    # get a session cookie by requesting the COOKIE_URL
    r = self.session.get(self.base_url + self.COOKIE_URL)

  def get(self,path):
    return xmltodict.parse(self.session.get(self.base_url + path).text).get('response',None)

def main():
  BTS_DATA = InitBTSDB()
  e3372 = HuaweiE3372()
  prev_head = 0
  a = 1
  while(1):
    #one shot subscribe
    msg = subscribe.simple("UAV/DOWN", hostname="localhost")
    #print("%s %s" % (msg.topic, msg.payload))
    data_decode=str(msg.payload.decode("utf-8","ignore"))
    data_uav=json.loads(data_decode) #decode json data
    data_uav.pop("interface")
    data_uav.pop("timestamp")

    #Taking needed params
    uav_lat = float(data_uav["gps_param"]["latitude"])
    uav_lon = float(data_uav["gps_param"]["longitude"])
    uav_head = float(data_uav["gps_param"]["head"])
    uav_speed = float(data_uav["gps_param"]["groundspeed"])
    uav_coo = (uav_lat, uav_lon)
    #print(uav_lat, uav_lon, uav_head, uav_speed)

    #Find Nearby BTSs
    BTS = CellFind(uav_coo, BTS_DATA)
    print("Nearest BTS is: " + BTS.site_name + " with Bearing " + str(round(BTS.bearing, 0)) + "°, UAV Head is " + str(uav_head) + "°")

    #Get bearing angles
    bts_bearing = round(BTS.bearing, 0)
    relative_bearing = bts_bearing - uav_head

    #Determine which antenna turn on
    if (uav_head>(prev_head + shiftdegree)) or (uav_head<(prev_head - shiftdegree)):
        if ((math.cos(math.radians(relative_bearing))>=0) and (math.sin(math.radians(relative_bearing))<0)):
            print('Front Left')
            Ant2()
            a = 2
        elif ((math.cos(math.radians(relative_bearing))>=0) and (math.sin(math.radians(relative_bearing))>=0)):
            print('Front Right')
            Ant1()
            a = 1
        elif ((math.cos(math.radians(relative_bearing))<0) and (math.sin(math.radians(relative_bearing))>=0)):
            print('Back Right')
            Ant4()
            a = 4
        elif ((math.cos(math.radians(relative_bearing))<0) and (math.sin(math.radians(relative_bearing))<0)):
            print('Back Left')
            Ant3()
            a = 3
        time.sleep(1)

    print("Antenna N° " + str(a)+ " is ON")
    #Change only if there is a head change
    prev_head = uav_head

    #Get LTE Quality Indexes
    for path in e3372.XML_APIS:
      #print("Path: "+str(path))
      for key,value in e3372.get(path).items():
        if key == "workmode":
          nettype = value
        if key == "rsrq":
          rsrq = value
        if key == "rsrp":
          rsrp = value
        if key == "rssi":
          rssi = value
        if key == "sinr":
          sinr = value
        if key == "band":
          band = value
        if key == "pci":
          pci = value

    print(nettype, rsrq, rsrp, rssi, sinr, band, pci) 
    #output = {"interface": "eth1", "timestamp": str(datetime.now()), "net_param": {"net_type": str(nettype), "rsrq": str(rsrq).replace("dB",""), "rsrp": str(rsrp).replace("dBm",""), "rssi": str(rssi).replace("dBm",""), "sinr": str(sinr).replace("dB",""), "band": str(band), "pci": str(pci)}, "uav_param": data_uav["uav_param"], "gps_param": data_uav["gps_param"]}
    #print(json.dumps(output))  
    print("LTE Check")
    print(datetime.now())
    time.sleep(interval)

if __name__ == "__main__":
  main()
