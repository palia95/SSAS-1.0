#!/usr/bin/env python
import sys
import pprint
import requests
import time
from datetime import datetime
import json
import math
import pandas as pd
import paho.mqtt.client as paho
import paho.mqtt.subscribe as subscribe
from pandas import DataFrame
import subprocess
import os
import warnings
import serial
warnings.filterwarnings("ignore")

#Launch MQTT_UAV
#subprocess.run(["python3", "Pi_FC/MQTT_UAV.py"])

#MQTT local broker connection
MQTT_BROKER_URL_RED = "xxxx"
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

    #Get LTE Quality Indexes
    lte_measurement = Scan_LTE()
    while(lte_measurement == 0):
        time.sleep(refresh)
        lte_measurement = Scan_LTE()

    print("lte_measurement")

    #Output to log
    #BTS_data = {"node_id": str(BTS.node_id), "site_name": str(BTS.site_name), "cell_lat": str(BTS.cell_lat), "cell_long": str(BTS.cell_long), "distance": str(BTS.distance), "bearing": str(BTS.bearing)}
    output = {"interface": "wwan0", "timestamp": str(datetime.now()), "net_param": lte_measurement["net_param"], "ant": str(a), "uav_param": data_uav["uav_param"], "gps_param": data_uav["gps_param"]}
    print(json.dumps(output))
    MQTT_NODE_RED_CLIENT.publish("GW/LTE", json.dumps(output))
    with open('LTE_Ant_Sierra_omni.json', 'a') as outfile:
        json.dump(output, outfile, ensure_ascii=False)
        outfile.write('\n')
        

if __name__ == "__main__":
  main()
