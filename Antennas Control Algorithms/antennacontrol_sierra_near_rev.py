#!/usr/bin/env python
import RPi.GPIO as GPIO
import sys
import pprint
import requests
import time
from geopy.distance import great_circle
from datetime import datetime
import json
import math
import pandas as pd
import paho.mqtt.client as paho
import paho.mqtt.subscribe as subscribe
from pandas import DataFrame
import subprocess
import os
import serial
import warnings
warnings.filterwarnings("ignore")

#Launch MQTT_UAV
#subprocess.run(["python3", "Pi_FC/MQTT_UAV.py"])

#MQTT local broker connection
MQTT_BROKER_URL_RED = "xxx"
MQTT_BROKER_PORT_RED = 1883
MQTT_BROKER_USERNAME_RED = "xxx"
MQTT_BROKER_PWD_RED = "xxx"

#MQTT Client Init
MQTT_NODE_RED_CLIENT = paho.Client()
MQTT_NODE_RED_CLIENT.connect(MQTT_BROKER_URL_RED, MQTT_BROKER_PORT_RED)
MQTT_NODE_RED_CLIENT.username_pw_set(MQTT_BROKER_USERNAME_RED, MQTT_BROKER_PWD_RED)
MQTT_NODE_RED_CLIENT.loop_start()

#Read interval
refresh = 0.5

#Set GPIO
GPIO.setmode(GPIO.BCM) # Choose BCM to use GPIO numbers instead of pin numbers
GPIO.setwarnings(False)
interval = 1

#Define pins
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
DELTA_GNSS = 0.1 #equivalent to about 5-6 km
DELTA_GNSS_INIT = 0.2 #equivalent to about 10-11 km
DELTA_POS = 0.0001 #equivalent to about 100 m
bts_latitude = 0
bts_longitude = 0
shiftdegree = 2 #degrees tollerated befoe switchin antenna

## configuration of the serial port comm
ser = serial.Serial(timeout=1)
ser.baudrate = 115200
ser.port = '/dev/ttyUSB0'
ser.open()

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
    BTS_DATA = BTS_DATA.drop(columns=['tech','mcc','mnc','lac_tac','cid','psc_pci','arfcn','azimuth','height','tilt_mech','tilt_el','cell_name','band'])
    BTS_DATA = BTS_DATA.round(4)

    BTS_DATA = BTS_DATA.drop_duplicates(subset=['node_id'])

    ###In order to limit processing, limit the DF to cells in 10km radius
    Nearby_Cells_Lat = abs(BTS_DATA['cell_lat']-uav_coo[0])<=DELTA_GNSS_INIT
    Nearby_Cells_Lon = abs(BTS_DATA['cell_long']-uav_coo[1])<=DELTA_GNSS_INIT

    BTS_DATA = BTS_DATA[Nearby_Cells_Lon]
    BTS_DATA = BTS_DATA[Nearby_Cells_Lat]

    print("BTS DB Loaded")
    print(BTS_DATA)

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
def CellFind_Distance(uav_coo, BTS_DATA):
    global DELTA_GNSS

    Nearby_Cells_Lat = abs(BTS_DATA['cell_lat']-uav_coo[0])<=DELTA_GNSS
    Nearby_Cells_Lon = abs(BTS_DATA['cell_long']-uav_coo[1])<=DELTA_GNSS

    BTS = BTS_DATA[Nearby_Cells_Lon]
    NEARBY_BTS = BTS[Nearby_Cells_Lat]

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

    #print(NEARBY_BTS)

    return(NEARBY_BTS)

#Find connected cell
def CellFind_Connected(uav_coo, eNB, NEARBY_BTS):
    cell = NEARBY_BTS.loc[NEARBY_BTS['node_id'] == eNB]
    print(cell)
    bts_coo = (float(cell['cell_lat']), float(cell['cell_long']))

    cell['distance'] = great_circle(uav_coo, bts_coo).m
    cell['bearing'] = calculate_bearing(uav_coo, bts_coo)

    return(cell)

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

def Scan_LTE():
    #Define varible to be read
    pcc_rxm_rssi = None
    pcc_rxm_rsrp = None
    pcc_rxd_rssi = None
    pcc_rxd_rsrp = None
    sinr = None
    rsrq = None
    cell_id = None
    lte_band = None
    lte_bw = None
    cell_id = None
    eNB = None
    id = None
    
    try:
        print("Read request begin " + str(datetime.now()))
        ser.write(b'AT!GSTATUS?\r\n')

        while True:
            #line=ser.readline().decode('utf-8')
            line = ser.readline().replace(b'\r\n',b'')
            line = line.decode('utf-8')

            if line:
            #line=line.decode('utf-8')
                #print(line)
                if line.startswith('PCC'):
                    if 'RxM' in line:
                        inner_line = line.split(':')
                        if 'PCC RxM RSRP' in inner_line[1]:
                            pcc_rxm_rssi = inner_line[1][:inner_line[1].index("PCC RxM RSRP")].strip()
                            pcc_rxm_rsrp = inner_line[2].strip()
                        #print('pcc_rxm_rssi:', pcc_rxm_rssi) #RSSI
                        #print('pcc_rxm_rsrp:', pcc_rxm_rsrp) #RSRP
                    if 'RxD' in line:
                        inner_line = line.split(':')
                        if 'PCC RxD RSRP' in inner_line[1]:
                            pcc_rxd_rssi = inner_line[1][:inner_line[1].index("PCC RxD RSRP")].strip()
                            pcc_rxd_rsrp = inner_line[2].strip()
                        #print('pcc_rxd_rssi:', pcc_rxd_rssi) #RSSI
                        #print('pcc_rxd_rsrp:', pcc_rxd_rsrp) #RSRP
                    
                elif line.startswith('SINR'):
                    inner_line = line.split(':')
                    sinr = inner_line[1].strip()
                    #print('sinr:', sinr) #SINR
                elif line.startswith('RSRQ'):
                    inner_line = line.split(':')
                    rsrq = inner_line[1][:inner_line[1].index("Cell ID")].strip()
                    cell_id = inner_line[2].strip()
                    #print('rsrq:', rsrq) #RSRQ
                    #print('cell id:', cell_id) #Cell ID
                elif line.startswith('LTE band'):
                    inner_line = line.split(':')
                    lte_band = inner_line[1][:inner_line[1].index("LTE bw")].strip()
                    lte_bw = inner_line[2].strip()
                    #print('lte_band:', lte_band) #LTE band
                    #print('lte_bw:', lte_bw) #LTE bw
                elif 'Cell ID' in line:
                    inner_line = line.split(':')
                    cell_id = inner_line[2].strip()
                    #print('cell_id:', cell_id) #Cell ID
                elif line.startswith('OK'):
                    print("Read request end " + str(datetime.now()))
                    break
            #else:
                #break


        mean_rsrp = round((int(pcc_rxm_rsrp)+int(pcc_rxd_rsrp))/2,1)
        mean_rssi = round((int(pcc_rxm_rssi)+int(pcc_rxd_rssi))/2,1)

        id = cell_id.split('(')
        cell_id = id[1].replace(')','')
        eNB = int(cell_id)//256

        measurement = {"timestamp": str(datetime.now()), "net_param": {"net_type": "LTE", "rsrq": str(rsrq), "rsrp": str(mean_rsrp), "rsrp_m": str(pcc_rxm_rsrp), "rsrp_d": str(pcc_rxd_rsrp), "rssi": str(mean_rssi), "rssi_m": str(pcc_rxm_rssi), "rssi_d": str(pcc_rxd_rssi), "sinr": str(sinr), "band": str(lte_band), "bandwidth": str(lte_bw).replace(" MHz",""), "cell_id": str(cell_id), "eNB": str(eNB)}}

        return measurement
    
    except:
        print("No signal")
        return 0

def main():
  prev_head = 0
  #Init BTS DB
  BTS_DATA = InitBTSDB()
  #First LTE scan
  lte_measurement = Scan_LTE()
  while(lte_measurement == 0):
    time.sleep(refresh)
    lte_measurement = Scan_LTE()
  a = 1

  while(1):
    #one shot subscribe
    msg = subscribe.simple("UAV/DOWN", hostname="localhost")
    #print("%s %s" % (msg.topic, msg.payload))
    print("got telemetry")
    data_decode=str(msg.payload.decode("utf-8","ignore"))
    data_uav=json.loads(data_decode) #decode json data
    data_uav.pop("interface")
    data_uav.pop("timestamp")

    #Taking needed params
    uav_lat = float(data_uav["gps_param"]["latitude"])
    uav_lon = float(data_uav["gps_param"]["longitude"])
    uav_head = float(data_uav["gps_param"]["head"])
    uav_speed = float(data_uav["gps_param"]["groundspeed"])
    #uav_coo = (uav_lat, uav_lon)

    #When there is no GPS use fake position
    #uav_lat = 45.0059289
    #uav_lon = 10.461025
    uav_coo = (uav_lat, uav_lon)
    #print(uav_lat, uav_lon, uav_head, uav_speed)
        #Find Nearby BTS
    BTS_Distance = CellFind_Distance(uav_coo, BTS_DATA)
    print("Nearest BTS is: " + BTS_Distance.iloc[0].site_name + " with ID " + str(BTS_Distance.iloc[0].node_id) + " at distance " + str(BTS_Distance.iloc[0].distance) + " m, with Bearing " + str(round(BTS_Distance.iloc[0].bearing, 0)) + "°, UAV Head is " + str(uav_head) + "°")

    BTS = BTS_Distance.iloc[0]
    print("Nearest BTS is " + BTS.site_name + " with ID " + str(BTS.node_id) + " at distance " + str(round(BTS.distance,2)) + " m, with Bearing " + str(round(BTS.bearing, 0)) + "°, UAV Head is " + str(uav_head) + "°")

    print(BTS)

    #Get bearing angles
    bts_bearing = round(BTS.bearing, 0)
    relative_bearing = bts_bearing - uav_head

    #Determine which antenna turn on
    if (uav_head>(prev_head + shiftdegree)) or (uav_head<(prev_head - shiftdegree)):
        if ((math.cos(math.radians(relative_bearing))>=0) and (math.sin(math.radians(relative_bearing))<0)):
            print('Front Left')
            #Ant2()
            Ant4()
            a = 2
        elif ((math.cos(math.radians(relative_bearing))>=0) and (math.sin(math.radians(relative_bearing))>=0)):
            print('Front Right')
            #Ant1()
            Ant3()
            a = 1
        elif ((math.cos(math.radians(relative_bearing))<0) and (math.sin(math.radians(relative_bearing))>=0)):
            print('Back Right')
            #Ant4()
            Ant2()
            a = 4
        elif ((math.cos(math.radians(relative_bearing))<0) and (math.sin(math.radians(relative_bearing))<0)):
            print('Back Left')
            #Ant3()
            Ant1()
            a = 3
        time.sleep(refresh)

    print("Antenna N° " + str(a)+ " is ON")
    #Change only if there is a head change
    prev_head = uav_head

    #Get LTE Quality Indexes
    lte_measurement = Scan_LTE()
    while(lte_measurement == 0):
        time.sleep(refresh)
        lte_measurement = Scan_LTE()

    #Output to log
    #BTS = BTS.iloc[0]
    BTS_data = json.loads(BTS.to_json())
    #BTS_data = {"node_id": str(BTS.node_id), "site_name": str(BTS.site_name), "cell_lat": str(BTS.cell_lat), "cell_long": str(BTS.cell_long), "distance": str(BTS.distance), "bearing": str(BTS.bearing)}
    output = {"interface": "wwan0", "timestamp": str(datetime.now()), "net_param": lte_measurement["net_param"], "ant": str(a), "BTS": BTS_data, "uav_param": data_uav["uav_param"], "gps_param": data_uav["gps_param"]}
    #print(json.dumps(output))
    MQTT_NODE_RED_CLIENT.publish("GW/LTEA", json.dumps(output))
    with open('LTE_Ant_Sierra_near.json', 'a') as outfile:
        json.dump(output, outfile, ensure_ascii=False)
        outfile.write('\n')
        

if __name__ == "__main__":
  main()
