# =================== IMPORTS ===================
import math
import sys
import os
import tkinter as tk
from tkinter import *
from tkinter import ttk
from tkinter import font
from time import time , sleep
import screeninfo
from datetime import datetime
from pytz import timezone, utc
import threading
import multiprocessing
from pymavlink import mavutil
from multiprocessing import Process
# ==================== Functions =====================
from logSaver import saveLoggs
# ==================== END OF IMPORTS ================
# ==================== GLOBAL VARIABLES ==============
global mavconn_established
mavconn_established = False

global PADAconn_established
PADAconn_established = False

global reportMavConn
reportMavConn = True

global mav_conn
mav_conn = None

global PADA_conn
PADA_conn = None

global mav_conn_string
global pada_conn_string
global window_call_string

if sys.platform == 'linux':
    mav_conn_string = '/dev/ttyUSB1'
    pada_conn_string = '/dev/ttyUSB0'
    window_call_string = ('./azure.tcl')
elif sys.platform == 'win32':
    mav_conn_string = 'COM17'
    pada_conn_string = 'COM18'
    window_call_string = r"./azure.tcl" # this is randy's path 
    # window_call_string = (os.getcwd() + '\\GS_Multithreading\\azure.tcl')
elif sys.platform == 'darwin':
    mav_conn_string = '/dev/tty.usbserial-B000IDOQ'
    pada_conn_string = '/dev/tty.usbserial-AK06O4AL'
    window_call_string = "/Users/labeast021/Desktop/GUI/Aero2024/azure.tcl"
else:
    print("ERROR: Unrecognized OS")

#####---------------------- GLOBAL VARIABLES ######################################################################
data = {
        'latitude': 0,
        'longitude': 0,
        'altitude': 0,
        'GPSaltitude' :0,
        'headingAngle': 0,
        'speed': 0, 
        'padaSpeed' : 0,  
        'signalStrength': 0,
    }

global sendPADALat
global sendPADALon

global color_pwm
color_pwm = 0

sendPADALat = 0
sendPADALon = 0

global logArray
logArray = []

global GPSLogger
GPSLogger = []

global triesForMission
triesForMission = 0

colorArray = ["BLUE", "ORANGE", "PURPLE", "RED", "YELLOW"]
colorPWM = [1750, 1650, 1450, 1550, 1850]

global detectedValue
detectedValue = 898

FIELD_ALTITUDE = 695 # in FEET

global errorCodes
errorCodes = [1200 , 1250 , 1950 ] # 1950 = Camera not detected, 1200 = Jetson not responding, 1250 = Jetson not ready

global errorRecieved
errorRecieved = []

global event_listener
event_listener = []
event_listener.append((1, datetime.now().strftime("%Y %m %d %H:%M:%S")))

global jetsonReady
global programRunning
global colorRecieved
global colorSent
global DAS_runnning
global DAS_stopped
global missionRecieved
global missionSent
global errorHandled 
global padaReadyToRelease
global start_time
global statusRecieved
global colorWaitRecieved 
global current_color
global sysvar
global sysstart
global sendMission
global timestamp
# ---------------------------------------------------------- END OF GLOBAL VARIABLES -------------------------------------------------------------------

# Get the primary monitor's screen resolution
screen = screeninfo.get_monitors()[0]
screen_width = screen.width
screen_height = screen.height
screen_geo = str(screen_width) + "x" + str(screen_height)
#############################################################################################
#CREATE TIMEZONE AND FORMATE FOR RECOREDER CREATION
utc = utc
pst = timezone('US/Pacific')
raw_TS = datetime.now(pst)

dateandtime = raw_TS.strftime("%b_%d_%Y_%H_%M_%S")
timestamp = raw_TS.strftime("[%H:%M:%S]")

# Create the flight data directory if it doesn't exist
flightdatadir = "flightdata"
if not os.path.exists(flightdatadir):
    os.makedirs(flightdatadir)

# Create the flight data log file
global current_theme
current_theme = "light"
# with open(flightdatapath, 'w') as fdr:
global window
window = tk.Tk()
window.title("Aero GUI")
# flightdatapath = os.path.join(flightdatadir, dateandtime + ".txt")
# with open(flightdatapath, 'w') as fdr:
#     pass  # Placeholder for further code
# THE COSMETICS AND SIZING OF THE WINDOW
def toggle_FS():
    window.geometry("800x600")
    window.focus_set()
pass
style = ttk.Style(window)
window.bind("<F10>", lambda e:toggle_FS())
window.bind("<Escape>", lambda e:window.destroy())
# window.call("source", "/home/salman/Desktop/AeroCode/Aero2024/azure.tcl")
window.call("source", window_call_string)
window.tk.call("set_theme", current_theme)
print("Screen Resolution: {}x{}".format(screen_width, screen_height))

window.geometry(screen_geo)
window.title("CSUN Aeronautics Ground Station")
# icon = PhotoImage(file=r"/home/salman/Desktop/AeroCode/Aero2024/app_icon.ico")
# icon = PhotoImage(file=r"Aero_Logo.ico")
# windosw.iconbitmap(r"Aero_Logo.ico")
# window.iconphoto(False, icon)

# MAIN SCREEN TITLE
label = ttk.Label(window, text="CSUN AERO 2024 GROUND", font=('None', screen_height//20))
label.pack(padx=(0.05*(screen_width)), pady=(0.05*(screen_height)))

# ADD SCROLLBAR
scrollbar = ttk.Scrollbar(window)
scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

# VERSION
version = "K.7.0"
gui_version = ttk.Label(window, text="Version: " + version, font=('None',10))
gui_version.place(x=(0.8*(screen_width)),y=(0.01*(screen_height)))

#CURRENT DATE AND TIME
timelabel = ttk.Label(text="Current Time", font=('None', 20))
timelabel.place(x=0.9*(screen_width), y=0.045*(screen_height))

datelabel = ttk.Label(text="Current Date", font=('None', 20))
datelabel.place(x=0.9*(screen_width), y=0.015*(screen_height))

raw_TS = datetime.now(pst)
date = raw_TS.strftime("%b %d %Y")
time = raw_TS.strftime("%H:%M:%S %p")
# formattime = raw_TS.strftime("%m-%d-%Y")
datelabel.config(text=date)
timelabel.config(text=time)

#---------------------------- LOG TEXT BOX CREATION ----------------------------
log = Text(window, height=int(0.025*(screen_height)), width=int(0.05*(screen_width)),xscrollcommand=True,yscrollcommand=True,state='normal')
log.place(x=0.64*(screen_width), y=0.53*(screen_height))
bold_font = font.Font(family='TkDefaultFont', size=10, weight='bold')

log.tag_configure("color1", foreground="green"  , font=bold_font)
log.tag_configure("color2", foreground="orange", font=bold_font) 
log.tag_configure("color3", foreground="red", font=bold_font)
log.tag_configure("color4", foreground="purple", font=bold_font)
log.tag_configure("font1", font=("TkDefaultFont", int(0.04*(screen_height))))
# log.tag_add("font1", "1.0", "end")

log.insert(1.0, f"{timestamp} Welcome to the CSUN Aeronautics Ground Station Log\n")
#---------------------------- END OF LOG TEXT BOX  ----------------------------

#---------------------------- logging the data   ----------------------------
def logSaverFunction():
    global log
    global logArray
    global event_listener
    global GPSLogger
    global timestamp
    log.insert(1.0, f"{timestamp} Saving Logs . . .\n" , "color4")
    saveLoggs(GPSLogger , event_listener)
    log.insert(1.0, f"{timestamp} Logs Saved\n" , "color4")

# --------------------------- Establishing connection to primary and PADA  . --------------------------
def request_message_interval(message_id, frequency_hz , device):
    global mav_conn
    global PADA_conn
    if device == 0:
        mav_conn.mav.command_long_send(
        mav_conn.target_system, mav_conn.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, 1000000 // frequency_hz, 0, 0, 0, 0, 0)
    else:
        PADA_conn.mav.command_long_send(
        PADA_conn.target_system, PADA_conn.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, 1000000 // frequency_hz, 0, 0, 0, 0, 0)

    return

def establish_mav_connection(report):
    global mavconn_established
    global mav_conn
    global mav_conn_string
    try:
        # Attempt to establish a MAVLink connection
        # mav_conn = mavutil.mavlink_connection('/dev/tty.usbserial-AK06O4AL', baud=57600) # for macOS
        mav_conn = mavutil.mavlink_connection(mav_conn_string, baud=115200)
        heartbeat = mav_conn.wait_heartbeat(timeout=5)
        
        if heartbeat is None:
            mavconn_established = False
            if report:
                log.insert(1.0, f"{timestamp} No heartbeat received. Connection not established to Orange Cube.\n", "color3")
            # log.insert(1.0, "No heartbeat received. Connection not established.\n", "color3")
            # print("No heartbeat received. Connection not established.")
        else:
            event_listener.append((5, datetime.now().strftime("%Y %m %d %H:%M:%S")))
            mavconn_established = True
            request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 10 , 0)  # Every 1 second for raw GPS data     
            request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10 , 0)  # Every 1 second for raw GPS data 
            request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10 , 0)  # Every 1 second for raw GPS data 
            request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 2 , 0)  
            log.insert(1.0, f"{timestamp} Connection successfully established To Orange Cube!\n", "color1")
            # print("Connection successfully established!")
        
    except Exception as e:
        if report:
            log.insert(1.0, f"{timestamp} Failed to establish connection to orange cube: {e}\n", "color3")
            
        # Handle cases where connection fails for other reasons
        # log.insert(1.0, f"Failed to establish connection: {e}\n", "color3")
        pass

establish_mav_connection(True)

def establish_PADA_connection(report):
    global PADAconn_established
    global PADA_conn
    global pada_conn_string

    try:
        # Attempt to establish a MAVLink connection
        # mav_conn = mavutil.mavlink_connection('/dev/tty.usbserial-AK06O4AL', baud=57600) # for macOS
        PADA_conn = mavutil.mavlink_connection(pada_conn_string, baud=115200)
        heartbeat = PADA_conn.wait_heartbeat(timeout=0.5)
        
        if heartbeat is None:
            PADAconn_established = False
            if report:
                log.insert(1.0, f"{timestamp} No heartbeat received. Connection not established to PADA.\n", "color3")
            # log.insert(1.0, "No heartbeat received. Connection not established.\n", "color3")
            # print("No heartbeat received. Connection not established.")
        else:
            event_listener.append((9, datetime.now().strftime("%Y %m %d %H:%M:%S")))
            PADAconn_established = True
            request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 10 , 1)  # Every 1 second for raw GPS data     
            log.insert(1.0, f"{timestamp} Connection successfully established To PADA!\n", "color1")
            # print("Connection successfully established!")
        
    except Exception as e:
        if report:
            log.insert(1.0, f"{timestamp} Failed to establish connection to PADA: {e}\n", "color3")
            
        # Handle cases where connection fails for other reasons
        # log.insert(1.0, f"Failed to establish connection: {e}\n", "color3")
        pass

establish_PADA_connection(True)
# --------------------------- ALL THE FUNCTIONALITY OF THE BUTTONS DOWN BELOW , NOTTE THESE ARE SEPERATE FROM THE THREADS . --------------------------

def start_system():
    global programRunning
    global event_listener
    global mav_conn

    mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component, mavutil.mavlink.MAV_CMD_DO_SET_SERVO,0, 2, 1200, 0, 0 , 0, 0, 0)
    sleep(0.2)
    # programRunning = True
    event_listener.append((3, datetime.now().strftime("%Y %m %d %H:%M:%S")))

launchstatvar = ttk.Label(window, text="", font=('None', 20))
launchstatvar.pack()

def launch():

    global padaReadyToRelease
    padaReadyToRelease = True

    global PADA_conn

    if padaReadyToRelease == True:
        mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component,mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 7 , 2100 , 0 , 0, 0, 0, 0)
        log.insert(1.0, f"{timestamp} PADA Launched at {data['altitude']}\n"  , "font1")
        # msg = mav_conn.recv_match(type = 'COMMAND_ACK', blocking=True)
        # print(msg) 
        sleep(1.5)
        mode = 'AUTO'  # Define the mode you want to set; for ArduPilot, AUTO mode for autonomous operation

        # Find the mode ID for AUTO mode
        mode_id = PADA_conn.mode_mapping()[mode]

        # Set the mode
        PADA_conn.mav.set_mode_send(
            PADA_conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
        launchstatvar.config(text=f"PADA Launched at {data['altitude']}\n")
    else:
        log.insert(1.0, f"{timestamp} DANGER ! No Mission on PADA to launch\n") 
    
# ================================= SENDING MISSION TO PADA ======================================================= 
def sendMissionToPADA(lat, lon , direction):
    print("in send mission 1 ")
    global pada_conn_string
    global PADA_conn
    global PADAconn_established
    global mav_conn
    global mav_conn_string
    
    from new_WayPoint_Generator import create_waypoints

    direction_code = 0
    if direction == 'South -> North': 
        direction_code = 1
    elif direction == 'west -> east':
        direction_code = 2
    elif direction == 'east -> west':
        direction_code = 3
    elif direction == 'north east -> south west':
        direction_code = 4
    elif direction == 'south west -> north east':
        direction_code = 5
    elif direction == 'north west -> south east':
        direction_code = 6
    elif direction == 'south east -> north west':
        direction_code = 7
    
    PADA_conn.close()
    PADA_conn = None
    PADAconn_established = False

    thread1 = threading.Thread(target = create_waypoints, args=(lat, lon , direction_code))
    thread1.start()
    thread1.join()

    sleep(1)
    try:
        # Attempt to establish a MAVLink connection
        PADA_conn = mavutil.mavlink_connection(pada_conn_string, baud=115200) # for macOS
        heartbeat = PADA_conn.wait_heartbeat(timeout=5)
        
        if heartbeat is None:
            log.insert(1.0, f"{timestamp} No heartbeat received. Connection not established to PADA.\n", "color3")
            # log.insert(1.0, "No heartbeat received. Connection not established.\n", "color3")
            print("No heartbeat received. Connection not established TO PADA.")
        else:
            PADAconn_established = True
            log.insert(1.0, f"{timestamp} Connection successfully established To PADA!\n", "color1")
            print("Connection successfully established! to PADA")
    
    except Exception as e:
        log.insert(1.0, f"{timestamp} Failed to establish connection to PADA: {e}\n", "color3")
    # Handle cases where connection fails for other reasons

    log.insert(1.0, f"{timestamp} Mission Sent To PADA !\n", "color1")

def stopDetection():
    global DAS_runnning
    
    DAS_runnning = True
    if DAS_runnning == True:
        print(event_listener)
        mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component,mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 3 , 1000 , 0 , 0, 0, 0, 0)
        sleep(0.5)
        log.insert(1.0, f"{timestamp} DAS STOPPED\n")    

def initialization():
    # global detectedValue
    # detectedValue = 898
    global reportMavConn
    reportMavConn = False
    global mavconn_established
    global padaReadyToRelease
    padaReadyToRelease = False
    global colorWaitRecieved
    colorWaitRecieved = False
    global statusRecieved
    statusRecieved = False
    global jetsonReady
    jetsonReady = False
    global programRunning
    programRunning = False
    global colorRecieved
    colorRecieved = False
    global colorSent
    colorSent = False
    global DAS_runnning
    DAS_runnning = False
    global DAS_stopped
    DAS_stopped = False
    global missionRecieved
    missionRecieved = False
    global missionSent
    missionSent = False
    global errorHandled 
    errorHandled = False 
    global current_color
    # global sendMission
    # global event_listener
    global start_time
    start_time = time
    
    # global sysvar
    # global sendMission
    # global sysstart
    # global sendMission
    # global window
    # sendMission = ttk.Button()
    # sysvar = StringVar()
    # sysvar.set("waiting for jetson ... ")
    # startsys = OptionMenu(window,sysvar ,"")
    # startsys.place(x=0.01*(screen_width),y=0.6*(screen_height), height=0.06*(screen_height), width=0.1*(screen_width))
    # sysstart = ttk.Button()
    # sysstart.place_forget()
    # sendMission.place_forget()
    # event_listener = []
    # GPSLogger = []

    global sysvar
    global SendMission
    sysvar = StringVar()
    sysvar.set("waiting for jetson ... ")
    startsys = OptionMenu(window,sysvar,"")
    startsys.place(x=0.01*(screen_width),y=0.6*(screen_height), height=0.06*(screen_height), width=0.1*(screen_width))
    # sysstart.place_forget()
    sendMission = ttk.Button(window, text="Send Mission")
    sendMission.place_forget()
    # sysstart.place_forget()  # or element.grid_forget() or element.place_forget()

def restartjetson(jetsonRunning):
        global detectedValue
        detectedValue = 898
        global event_listener
        global GPSLogger
        global triesForMission

        mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component,mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 4 , 1900 , 0 , 0, 0, 0, 0)
        log.insert(1.0, "=============================================\n")
        sleep(5)
        triesForMission = 0
        event_listener = []
        GPSLogger = []
        event_listener.append((2, datetime.now().strftime("%Y %m %d %H:%M:%S")))

        initialization()

# --------------------------- ALL THE FUNCTIONALITY OF THE BUTTONS ABOVE , NOTTE THESE ARE SEPERATE FROM THE THREADS . --------------------------
# ========================================THREADS AND FUNCTIONS THAT WILL REPEAT ARE BELOW ===================================
def uptadeTime():
    raw_TS = datetime.now(pst)
    time = raw_TS.strftime("%H:%M:%S %p")
    timelabel.config(text=time)
    window.after(1000, uptadeTime)

def getHeartbeat():
    global current_color
    global start_time
    global jetsonReady
    global colorRecieved
    global colorSent
    global DAS_runnning
    global missionRecieved
    global sysstart
    global event_listener
    global statusRecieved
    global colorWaitRecieved
    global errorCodes
    global errorRecieved
    global errorHandled
    global triesForMission
    global detectedValue
    global GPSLogger
    global PADAconn_established
    global PADA_conn
    global sendPADALat
    global sendPADALon
    global timestamp

    while True:
        GPSTuple = ()
        # get current time
        timestamp = datetime.now().strftime("[%H:%M:%S]")
        if PADAconn_established == False or PADA_conn == None:
            establish_PADA_connection(False)

        if mavconn_established == True and mav_conn is not None:
            globalPosition = mav_conn.recv_match(type = 'GLOBAL_POSITION_INT', blocking=True)
            if globalPosition is not None:
                data['altitude'] = ((globalPosition.relative_alt * 3.2808 ) / 1000 ) #Chnage to airfield alt
                data['headingAngle'] = globalPosition.hdg / 100
                data['latitude'] = globalPosition.lat / 1e7
                data['longitude'] = globalPosition.lon / 1e7
            # ============================================
            # vfrHud = mav_conn.recv_match(type = 'VFR_HUD', blocking=True)
            # if vfrHud is not None:
            #     data['speed'] = vfrHud.airspeed
            # ============================================
            GPSRAW = mav_conn.recv_match(type = 'GPS_RAW_INT', blocking=True)
            if GPSRAW is not None:
                data['GPSaltitude'] = ((GPSRAW.alt * 3.2808) /1000) - 700
            # ============================================
            # powerStatus = mav_conn.recv_match(type = 'POWER_STATUS', blocking=True)
            # if powerStatus is not None:
            #     data['powerstatus'] = powerStatus.Vcc  / 1000
            # ============================================
            if PADAconn_established == True and PADA_conn is not None:
                try:
                    padaSpeed= PADA_conn.recv_match(type = 'VFR_HUD', blocking=True, timeout=.3 )
                    data['speed'] = padaSpeed.airspeed
                except:
                    print("error getting the speed")
            # ============================================
            # Attitude= mav_conn.recv_match(type = 'ATTITUDE', blocking=True )
            # yaw = Attitude.yaw
            # yaw_degree = math.degrees(yaw)
            # if yaw_degree < 0:
            #     yaw_degree += 360
            # print(yaw_degree)
            # ============================================
            # signal = mav_conn.recv_match(type = 'RADIO_STATUS', blocking=True , timeout=1)
            # if signal is not None:
            #     # data['signalStrength'] = signal.rssi
            #     print(signal.rssi)
            # ============================================ 
            GPSTuple = (data['latitude'], data['longitude'], data['altitude'], data['headingAngle'])
            GPSLogger.append(GPSTuple)

            servo_msg = mav_conn.recv_match(type='SERVO_OUTPUT_RAW' ,blocking=True)
            # print(servo_msg)
            if not any(existing_code == servo_msg.servo5_raw for existing_code, _ in event_listener) and servo_msg.servo5_raw != 0:
                if servo_msg.servo5_raw == 1500 and jetsonReady == False and servo_msg.servo5_raw != 0:
                    event_listener.append((servo_msg.servo5_raw , datetime.now().strftime("%Y %m %d %H:%M:%S")))
                    jetsonReady = True
                    sysvar.set("Jetson Ready \u2713")  # \u2713 is the unicode for a check mark symbol
                    # sysstart.pack() 
                    sysstart.place(x=0.12*(screen_width), y=0.6*(screen_height), height=0.06*(screen_height), width=0.1*(screen_width))
                    # log.tag_configure("color1", foreground="green")  # Change "red" to the desired color
                    log.insert(1.0, f"{timestamp} code {servo_msg.servo5_raw} : Jetson ready \n", "color1")

            
            if not any(existing_code == servo_msg.servo1_raw for existing_code, _ in event_listener) and servo_msg.servo1_raw != 0:

                event_listener.append((servo_msg.servo1_raw , datetime.now().strftime("%Y %m %d %H:%M:%S")))

                if any(existing_code == servo_msg.servo1_raw for existing_code in errorCodes) :
                    if servo_msg.servo1_raw not in errorRecieved:
                        errorHandled = True if errorHandled == False else ()
                        errorRecieved.append(servo_msg.servo1_raw)
                        log.insert(1.0, f"{timestamp} ERROR! CODE: {servo_msg.servo1_raw}\n", "color3")
                        sysvar.set("Jetson Stopped ")

                elif servo_msg.servo1_raw == 1440 and colorRecieved == True:
                    colorRecieved = False
                    log.insert(1.0, f"{timestamp} code {servo_msg.servo1_raw} : Detection Stopped \n", "color1")
                
                elif servo_msg.servo1_raw == 1600 and colorWaitRecieved == False:
                    colorWaitRecieved = True
                    log.insert(1.0, f"{timestamp} code {servo_msg.servo1_raw} : Jetson waiting for color ... \n", "color1")
                    
                elif servo_msg.servo1_raw ==1900 and statusRecieved == False:
                    statusRecieved = True
                    log.insert(1.0, f"{timestamp} code {servo_msg.servo1_raw} : Initialization on jetson was successful \n", "color1")
                    
                elif servo_msg.servo1_raw == detectedValue :
                    log.insert(1.0, f"{timestamp} code {servo_msg.servo1_raw} : Detected a Target  \n", "color1")
                    
                elif servo_msg.servo1_raw == 800 :
                    log.insert(1.0, f"{timestamp} code {servo_msg.servo1_raw} : Did not detect a target  \n", "color2")
                    
                elif servo_msg.servo1_raw in colorPWM and colorRecieved == False:
                    colorRecieved = True
                    current_color = colorArray[colorPWM.index(servo_msg.servo8_raw)]
                    # rs.set(f"DAS Runnning | {current_color} ")
                    log.insert(1.0, f"{timestamp} code {servo_msg.servo1_raw} : DAS running with color : {current_color}\n", "color1")
                    DAS_runnning = True

                if any(existing_code == 1440 for existing_code, _ in event_listener) and missionRecieved == False: 
                    sleep(2)
                    mav_conn.waypoint_request_list_send()
                    sleep(1)
                    
                    # mav_conn.mission_request_list_send(mav_conn.target_system, mav_conn.target_component)
                    servo_msg_for_mission = mav_conn.recv_match(type='MISSION_COUNT', blocking=True, timeout=2)

                    if servo_msg_for_mission is None and triesForMission < 3:
                        log.insert(1.0, f"{timestamp} Failed to receive mission count {triesForMission}\n", "color3")
                        for i in range(len(event_listener) - 1, -1, -1):  # Start from the last index, go to 0
                            if 1440 in event_listener[i]:
                                del event_listener[i]  # Use del to remove the item at index i
                                triesForMission += 1
                                break
                    elif servo_msg_for_mission:
                        log.insert(1.0, f"{timestamp} Mission count received: {servo_msg_for_mission.count}\n", "color1")
                        missionRecieved = True
                        
                        for seq in range(servo_msg_for_mission.count):
                            while True:                   # Request the specific waypoint
                                mav_conn.waypoint_request_send(seq)
                                waypoint = mav_conn.recv_match(type='MISSION_ITEM', blocking=True , timeout=1)
                                if waypoint:
                                    lat = 0
                                    lon = 0
                                    print(f"Waypoint {seq}: Lat {waypoint.x}, Lon {waypoint.y}, Alt {waypoint.z}")
                                    print("Full precision:", f"Lat {format(waypoint.x, '.8f')}, Lon {format(waypoint.y, '.8f')}")
                                    lat = waypoint.x  # Latitude
                                    lon = waypoint.y  # Longitude
                                    sendPADALat = waypoint.x
                                    sendPADALon = waypoint.y
                                    log.insert(1.0, f"{timestamp} Mission Received coordinates for target -> lat: {lat}, lon: {lon}\n")
                                    print(f"Landing waypoint found: Latitude = {lat}, Longitude = {lon}")
                                    break

        elif mavconn_established == False or mav_conn is None:
            establish_mav_connection(False)
        sleep(0.5)

def creategui():

    global sendPADALat
    global sendPADALon

    # ADD SETTINGS
    def change_screen(screen_name):
        global current_theme
        if screen_name == "change_theme":
            if current_theme == "dark":
                window.destroy()
                current_theme = "light"
                window.after(1)
                creategui()
            else:
                window.destroy()
                current_theme = "dark"
                window.after(1)
                creategui()
        pass

    pada_arm = StringVar()
    pada_arm.set("Which Direction?")
    pada_sel = OptionMenu(window, pada_arm, 'North -> South','South -> North','west -> east','east -> west','north east -> south west','south west -> north east','north west -> south east','south east -> north west')
    pada_sel.place(x=0.28*(screen_width), y=0.7*(screen_height), height=0.06*(screen_height), width=0.1*(screen_width))  
    sendMission = ttk.Button(window, text="Send Mission", command=lambda: sendMissionToPADA( sendPADALat , sendPADALon , pada_arm.get()))
    sendMission.place(x=0.4*(screen_width), y=0.7*(screen_height), height=0.06*(screen_height), width=0.1*(screen_width))



    menu = ttk.Menubutton(window, text="Utility Menu")
    menu.place(x=(0.01*(screen_width)), y=(0.01*(screen_height)))
    settings_menu = tk.Menu(menu, tearoff=False)
    menu['menu'] = settings_menu

    # Add options to the settings menu
    settings_menu.add_command(label="Settings", command=lambda: change_screen("Settin"))
    settings_menu.add_command(label="I DO NOTHING", command=lambda: change_screen(""))
    settings_menu.add_command(label="I CHANGE THEMES", command=lambda: change_screen("change_theme"))
    
    #date_var = StringVar()

    #DASHBOARD CREATION, MAKING A TABLE TO PLACE INFO INTO
    dashboard = ttk.LabelFrame(window)
    dashboard.columnconfigure(0, weight=1)
    dashboard.columnconfigure(1, weight=1)
    dashboard.columnconfigure(2, weight=1)

    #LABEL CREATION THIS EDITS NAMES ONLY/ VAR variants are the data that will be displayed default is 0

    #ALTITUDE
    altitude = ttk.Label(dashboard, text="Altitude", font=('None', 20))
    altitude.grid(row=0,column=0, sticky="news", padx=0.001*(screen_width))

    #ALTTITUDE VAR
    altvar = ttk.Label(dashboard, text="0", font=('None', 90))
    altvar.grid(row=1,column=0,sticky="news", padx=0.001*(screen_width))
# =======================================
    #ALTITUDE
    GPSaltitude = ttk.Label(dashboard, text="GPS Altitude", font=('None', 20))
    GPSaltitude.grid(row=0,column=4, sticky="news", padx=0.001*(screen_width))

    #ALTTITUDE VAR
    GPSaltvar = ttk.Label(dashboard, text="0", font=('None', 90))
    GPSaltvar.grid(row=1,column=4,sticky="news", padx=0.001*(screen_width))
# ===========================================
    #HEADING
    heading = ttk.Label(dashboard, text="Heading", font=('None', 20))
    heading.grid(row=0,column=1, sticky="news", padx=0.001*(screen_width))

    #HEADING VAR
    hdgvar = ttk.Label(dashboard, text="0", font=('None', 90))
    hdgvar.grid(row=1,column=1,sticky="news")

    #AIRSPEED
    airspeed = ttk.Label(dashboard, text="Airspeed", font=('None', 20))
    airspeed.grid(row=0,column=2, sticky="news")

    #AIRSPEED VAR
    spdvar = ttk.Label(dashboard, text="0", font=('None', 90))
    spdvar.grid(row=1,column=2,sticky="news")


    # airspeed_text = tk.StringVar()
    # arsvar = ttk.Label(dashboard, textvariable=airspeed_text)
    #arsvar = ttk.Label(dashboard, text="0.00", font=('None', 100))
    #arsvar.grid(row=1,column=2,sticky="news")

    #PACK TO DISPLAY
    dashboard.pack(fill='x')

    #2ND DATA GROUP

    #2ND ROW CREATION, MAKING A TABLE TO PLACE INFO INTO
    dashboard2line = ttk.LabelFrame(window)
    dashboard2line.columnconfigure(0, weight=1)
    dashboard2line.columnconfigure(1, weight=1)
    dashboard2line.columnconfigure(2, weight=1)


    #LABEL CREATION THIS EDITS NAMES ONLY

    #LATTTITUDE
    latitude = ttk.Label(dashboard2line, text="Latitude", font=('None', 20))
    latitude.grid(row=0,column=0, sticky="news", padx=0.0001*(screen_width))

    #LONGITUDE
    longitude = ttk.Label(dashboard2line, text="Longitude", font=('None', 20))
    longitude.grid(row=0,column=1, sticky="news")

    # #POWER STATUS
    # powerstatus = ttk.Label(dashboard2line, text="Voltage Status", font=('None', 20))
    # powerstatus.grid(row=0,column=2, sticky="news",)

    #INITIAL DATA VAR CREATION, DEFAULT IS 0

    #LATITUDE VAR
    latvar = ttk.Label(dashboard2line, text="0", font=('None', 50))
    latvar.grid(row=1,column=0,sticky="news", padx=0.0009*(screen_width))

    #LONGITUDE VAR
    longvar = ttk.Label(dashboard2line, text="0", font=('None', 50))
    longvar.grid(row=1,column=1,sticky="news")

    #POWER STATUS VAR
    # pwrvar = ttk.Label(dashboard2line, text="0", font=('None', 50))
    # pwrvar.grid(row=1,column=2,sticky="news")

    #PACK TO DISPLAY
    dashboard2line.pack(fill='x')


    #DASHBOARD 3 LAUNCH SATUS CREATION
    dashboard3 = ttk.LabelFrame(window)
    dashboard3.columnconfigure(0, weight=1)
    dashboard3.columnconfigure(1, weight=1)
    dashboard3.columnconfigure(2, weight=1)

    #DAS3 LABEL CREATION THIS EDITS NAMES ONLY

    #COLOR SENT TO PRIMARY

    primcolor = ttk.Label(dashboard3, text="Color in Primary", font=('None', 12))
    primcolor.grid(row=0,column=0, sticky="news", padx=0.0001*(screen_width))

    #ARMED STATUS

    armedstat = ttk.Label(dashboard3, text="Armed Status", font=('None', 12))
    armedstat.grid(row=0,column=1, sticky="news")

    #LAUNCHED STATUS

    launchstat = ttk.Label(dashboard3, text="Launched Status", font=('None', 12))
    launchstat.grid(row=0,column=2, sticky="news", padx=0.0001*(screen_width))


    #DAS3 VAR CREATION

    #Prim Color VAR
    primcolorvar = ttk.Label(dashboard3, text="null", font=('None', 12))
    primcolorvar.grid(row=1,column=0,sticky="news", padx=0.0001*(screen_width))

    #AMRED STATUS VAR
    armedstatvar = ttk.Label(dashboard3, text="null", font=('None', 12))
    armedstatvar.grid(row=1,column=1,sticky="news")

    #LAUNCHED STATUS VAR
    launchstatvar = ttk.Label(dashboard3, text="Not Launched", font=('None', 12))
    launchstatvar.grid(row=1,column=2,sticky="news",padx=0.0001*(screen_width))


    #PACK TO DISPLAY
    dashboard3.place(x=0.01, y=0.52*(screen_height))

    # #################################################################################
    global das_arm_status
    das_arm_status = 0

    #run in headlass mode
    armheadless = StringVar()
    rs = StringVar()
    rs.set("DAS has not been started")
    runstop = OptionMenu(window, rs, 'STOP',"Don't STOP")
    runstop.config(cursor="hand2")  # Change cursor spec to "hand2
    runstop.place(x=0.01*(screen_width), y=0.7*(screen_height), height=0.06*(screen_height), width=0.1*(screen_width))
    # rs.trace('w', update_das_arm_status)  
    armheadless = ttk.Button(window, text="STOP DAS", command=stopDetection)
    armheadless.place(x=0.12*(screen_width), y=0.7*(screen_height), height=0.06*(screen_height), width=0.1*(screen_width))

    ############################################################################################################################

    # Jetson to ground

    dashboard4 = ttk.LabelFrame(window)
    
    dashboard4.columnconfigure(0, weight=1)
    dashboard4.columnconfigure(1, weight=1)

    #LABEL CREATION THIS EDITS NAMES ONLY
    jetstatname = ttk.Label(dashboard4, text="Jetson Status", font=('None', 12))
    jetstatname.grid(row=0,column=0,sticky="news", padx=10)
    jetstat = ttk.Label(dashboard4, text="Status", font=('None', 12))
    jetstat.grid(row=1,column=0,sticky="news", padx=10)

    dashboard4.place(x=0.3*(screen_width), y=0.52*(screen_height))
    #Var Ints

    # ##################################################################################################################################

    # jetsonreset = ttk.Button(window, text="Reset Jetson", command= restartjetson) 
    jetsonreset = ttk.Button(window, text="Reset Jetson", command=lambda: restartjetson(DAS_runnning)) 
    jetsonreset.place(x=0.12*(screen_width), y=0.8*(screen_height),height=0.06*(screen_height), width=0.1*(screen_width))
    
    # #################################################################################################################################################
    global sysstart
    

    def selectColor():
        global colorRecieved
        global detectedValue
        if color_pwm != 0:
            colorRecieved = False
            mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component,mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 8 , color_pwm, 0, 0, 0, 0, 0)
            log.insert(1.0, f"{timestamp} Color {selected_color.get()} is sent to PA\n")
            detectedValue += 2
        
    def update_selected_color(*args):
        global color_pwm

        selected_color.set(color_options.get())
        if selected_color.get() == "RED":
            color_pwm = 1550
            color_sel.configure(background='red', foreground='black')
            log.insert(1.0, f"{timestamp} RED is selected\n")
        elif selected_color.get() == "BLUE":
            color_pwm = 1750
            color_sel.configure(background='blue', foreground='white')
            log.insert(1.0, f"{timestamp} BLUE is selected\n")
        elif selected_color.get() == "ORANGE":
            color_pwm = 1650
            color_sel.configure(background='orange', foreground='black')
            log.insert(1.0, f"{timestamp} ORANGE is selected\n")
        elif selected_color.get() == "YELLOW":
            color_pwm = 1850
            color_sel.configure(background='yellow', foreground='black')
            log.insert(1.0, f"{timestamp} YELLOW is selected\n")
        elif selected_color.get() == "PURPLE":
            color_pwm = 1450            
            color_sel.configure(background='purple', foreground='black')    
            log.insert(1.0, f"{timestamp} PURPLE is selected\n")
        
    selected_color = StringVar()
    # DROPDOWN MENU
    color_options = StringVar()
    color_options.set("SELECT A COLOR")
    color_sel = OptionMenu(window, color_options, 'RED','BLUE','YELLOW','ORANGE','PURPLE')
    color_sel.config(cursor="hand2")  # Change cursor spec to "hand2"
    color_sel.place(x=0.4*(screen_width), y=0.6*(screen_height), height=0.06*(screen_height), width=0.1*(screen_width))
    # Add a label to display the selected color
    color_options.trace('w', update_selected_color)
    # selected_color_label = Label(window, textvariable=selected_color)
    # selected_color_label.pack() 
    selected_color_button = ttk.Button(window, text="Send the color to PA", command=selectColor)
    selected_color_button.place(x=0.51*(screen_width), y=0.6*(screen_height), height=0.06*(screen_height), width=0.1*(screen_width)) 

    # #################################################################################################################################################3
    global readyToLaunch
    readyToLaunch = False

    # pada_arm = StringVar()
    # pada_arm.set("Which Direction?")
    # pada_sel = OptionMenu(window, pada_arm, 'North -> South','South -> North')
    # pada_sel.place(x=0.28*(screen_width), y=0.7*(screen_height), height=0.06*(screen_height), width=0.1*(screen_width))  
    # pada_arm.trace('w', arm_pada)   

    # PADA RELEASE BUTTON
    release = ttk.Button(window, text="PADA LAUNCH" , command=launch)
    release.place(x=0.51*(screen_width), y=0.7*(screen_height), height=0.06*(screen_height), width=0.1*(screen_width))

    # ##################################################################################################################################################

    SaveLoggButton = ttk.Button(window, text="Save The Loggs" , command=logSaverFunction)
    SaveLoggButton.place(x=0.51*(screen_width), y=0.8*(screen_height), height=0.06*(screen_height), width=0.1*(screen_width))
    # START SYSTEM AESTHETIC DROPBOX MENU  
        
    # sysvar = StringVar()
    sysstart = ttk.Button(window, text="START DAS" , command=(start_system))
    
    # sysvar.set("waiting for jetson ... ")
    # startsys = OptionMenu(window,sysvar ,"")
    # startsys.place(x=0.01*(screen_width),y=0.6*(screen_height), height=0.06*(screen_height), width=0.1*(screen_width))

# --------------------------------------------------------------------------------------------------------------

    initialization()

# --------------------------------------------------------------------------------------------
    def updateDashInfoFunction():
        raw_TS = datetime.now(pst)
        time = raw_TS.strftime("%H:%M:%S %p")
        timelabel.config(text=time)
        # window.after(1000, uptadeTime)
        GPSaltvar.config(text="{:.1f} ft".format(data['GPSaltitude']))
        altvar.config(text="{:.2f} ft".format(data['altitude']))  #### LETS ADD A UNIT TO THIS ####
        hdgvar.config(text="{:.2f}".format(data['headingAngle']))
        latvar.config(text="{:.10f}".format(data['latitude']))
        longvar.config(text="{:.10f}".format(data['longitude']))
        spdvar.config(text="{:.2f}".format(data['speed']))
        # pwrvar.config(text="{:}".format(data['powerstatus']))
        window.after(500, updateDashInfoFunction)

    updateTimes = threading.Thread(target=uptadeTime , daemon=True)
    updateTelemInfo = threading.Thread(target=getHeartbeat , daemon=True)
    # updateDashInfo = threading.Thread(target=updateDashInfoFunction , daemon=True) THERE REASON IS THAT YOU CANNOT UPDATE THE GUI IN A THREAD

    updateTimes.start()
    updateTelemInfo.start()
    # updateDashInfo.start()

    updateDashInfoFunction()
    window.mainloop()

creategui()




































# ========================================================================================================================================================================
# total_waypoints = servo_msg_for_mission.count
                        # if total_waypoints > 0:
                        #     # Directly request the last waypoint, assuming 0-based indexing
                        #     last_waypoint_index = total_waypoints - 1
                        #     mav_conn.waypoint_request_send(last_waypoint_index)
                        #     last_waypoint = mav_conn.recv_match(type='MISSION_ITEM', blocking=True, timeout=5)
                            
                        #     if last_waypoint is not None:
                        #         log.insert(1.0, f"Last Waypoint: Latitude={last_waypoint.x}, Longitude={last_waypoint.y}\n")
                        #         # Process the waypoint as needed
                        #         lat = last_waypoint.x
                        #         lon = last_waypoint.y
                        #         # Example: Update the GUI or trigger further actions based on the lat/lon
                        #         sendMission = ttk.Button(window, text="Send Mission", command=lambda: sendMissionToPADA(lat, lon))
                        #         sendMission.place(x=0.12*(screen_width), y=0.8*(screen_height), height=0.06*(screen_height), width=0.1*(screen_width))
                        #     else:
                        #         log.insert(1.0, "Failed to receive the last waypoint\n", "color3")
                        
                        # sendMission = ttk.Button(window, text="Send Mission", command=lambda: sendMissionToPADA(lat, lon))
                        # sendMission.place(x=0.12*(screen_width), y=0.8*(screen_height), height=0.06*(screen_height), width=0.1*(screen_width))  
# ========================================================================================================================================================================
                        # while triesForMission < 6:
                        #     # Wait for a MISSION_ITEM message
                        #     message = mav_conn.recv_match(type='MISSION_ITEM', blocking=True , timeout=2)
                        #     if message:
                        #         # Check if this mission item is a landing waypoint
                        #         if message.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
                        #             # Extract the coordinates
                        #             lat = message.x  # Latitude
                        #             lon = message.y  # Longitude
                        #             print(f"Landing waypoint coordinates: Latitude = {lat}, Longitude = {lon}")
                        #             log.insert(1.0, f"Landing waypoint coordinates: Latitude = {lat}, Longitude = {lon}\n", "color1")
                        #             break  # Exit theit the loop if you only care about the f
                        #     triesForMission += 1
                        # if triesForMission == 6:
                        #     log.insert(1.0, "Failed to receive landing waypoint coordinates\n", "color3")    
                        # for i in range(servo_msg_for_mission.count):
                        #     print("hereee 4 ")
                        #     mav_conn.waypoint_request_send(i)
                        #     waypoint = mav_conn.recv_match(type='MISSION_ITEM', blocking=True)
                        #     print(waypoint)
                        #     # print(f"Waypoint {i + 1}: Latitude={waypoint.x}, Longitude={waypoint.y}, Altitude={waypoint.z}")
                        #     if i == servo_msg_for_mission.count - 1:
                        #         print("hereee 5 ")
                        #         current_color = colorArray[colorPWM.index(servo_msg.servo8_raw)]
                        #         log_text = f"Mission Received coordinates for target {current_color} -> lat: {waypoint.x}, lon: {waypoint.y}\n"
                        #         log.insert(1.0, log_text)
                        #         lat = waypoint.x
                        #         lon = waypoint.y