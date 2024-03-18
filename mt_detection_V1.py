# ** General Packages ** #
from os import path, execl, getcwd
from sys import executable
from sys import argv
# import time
from time import sleep, time, strftime, localtime 

# ** Packages for image Processing ** #
# torch packages
from torch.cuda import device_count, get_device_name, get_device_properties, is_available
from torch.backends.cudnn import version as cudnn_version
from torch.version import cuda as torch_cuda_version

from ultralytics import YOLO

# cv2 packages
from cv2 import VideoCapture, imshow, waitKey, destroyAllWindows, putText, FONT_HERSHEY_SIMPLEX, LINE_AA
# from Yolov8_organized.tensorRT_model.Multithreading.MIPI_Camera.utils import ArducamUtils
# imutils packages
from imutils import resize as imutils_resize

# ** Packages for math and data manipulation ** #
from numpy import array as np_array
from numpy import mean as np_mean
from numpy import set_printoptions as np_set_printoptions
from numpy import where as np_where

# ** Packages for telemetary information ** #
# from pymavlink import mavutil
from pymavlink import mavutil
from pymavlink.mavutil import mavlink_connection
from pymavlink.dialects.v20.common import MAV_CMD_DO_SET_SERVO

# ** Packages for multiprocessing and multithreading ** #
# import threading as th
from threading import Thread
from threading import Event 

# ** Packages and Functions for Gps calculation ** #
from GPS import *
# from gps_reader import *
from frame_processing import *
# from waypoint_2_mav import waypoint_gen_format
# from upload_mission import execute_upload
# ** Packages for file operations ** #
# from data_logger import *
from data_logger import perform_file_operations


# from extra_functions import*
###################################################################################################################
######## Global Variables #########################################################################################

flightControllerString = "/dev/ttyUSB0"
mav_conn = None

# MAVLink connection
try:
    mav_conn = mavlink_connection(flightControllerString, baud=57600)
    mav_conn.wait_heartbeat(timeout=30)
    print("Mavlink connection established and Heartbeat recieved (Timed out after 30 seconds))")
    
    sleep(3)

except Exception as e:
    print(f"Error creating Mavlink connection and/or recieving the heartbeat: {str(e)}")
    exit()

# TensorRT_Engine_Path = "/home/aero/Desktop/YOLOv8/Yolov8_organized/tensorRT_model/Multithreading/pt2trt/best (1).engine"
TensorRT_Engine_Path = "/home/aero/Desktop/crontab/Models/best (1).engine"

''' The indexes of each pwm code corresponds to the class number for the colors
    So for example blue is in index 0 in the array so the class for color blue is also 0
'''
COLORS = ["Blue" , "Orange" , "Purple" , "Red" , "Yellow" ]
COLORS_PWM = [1750 , 1650 , 1450 , 1550 , 1850]
telem_info = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # lat, lon, hdg, pitch, roll, alt


lat_conversion = 2.745e-6 #Apollo Field
lon_conversion = 3.3095732e-6 #Apollo Field

# Global variables for latitude, longitude, and altitude
global global_latitude 
global global_longitude 
global global_altitude 

global_latitude, global_longitude, global_altitude = None, None, None

global coordinates
coordinates = [[] , [] , [] , [] , []]

###################################################################################################################

global selectedColor
selectedColor = 0
global stopDetection
stopDetection = False
global DAS_started
DAS_started = False
global colorSelectedMsg
colorSelectedMsg = False
global color_class
color_class = -1
global detected
detected = False
global detectedValue

###################################################################################################################
######## All Functions  ###########################################################################################


# ** Function that will check to make sure everything is ready for the process ** #  
def initial_checkup(result_event):
    result_event.clear()
    
    cap = VideoCapture(0)
    if path.exists(flightControllerString) and cap.isOpened() and is_available() :
        cap.release()
        result_event.set()

        print("Telemetary connection is established \ncamera is ready \nGPU (CUDA) is available ")
        print("Number of GPUs :", device_count())
        print("Current GPU :", get_device_name(0))
        print("Current GPU poperties :", get_device_properties(0))
        print("Current cudnn :", cudnn_version())
        print("cuda version :" , torch_cuda_version)   
               
    else:
        if not cap.isOpened():
            mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component, MAV_CMD_DO_SET_SERVO, 0, 1 , 1200 , 0 , 0, 0, 0, 0)
            print("Camera is not avaialble, please check the connection and try again")
            sleep(1)
        if not path.exists(flightControllerString):
            print("Telemetary connection is not available, please check the connection and try again (USB not connected)")   
        if not is_available():
            mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component, MAV_CMD_DO_SET_SERVO, 0, 1 , 1250 , 0 , 0, 0, 0, 0)
            print("CUDA (GPU) is not available, please check and try again") 
            sleep(1)    
         
        result_event.clear()
    
# ** Function that will detect the objects and calculate the gps coordinates ** #  
def frame_retrival_and_detection(coordinates):
    
    global stopDetection
    global selectedColor
    global DAS_started
    global colorSelectedMsg
    global color_class
    global detected
    global detectedValue

    model = YOLO(TensorRT_Engine_Path, task='detect') # Change this to the path of the TensorRT engine file
    cap = VideoCapture(0)

    while stopDetection is not True:
    # ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        # start_time = time() # Comment when we test in headlessS
    # ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
        if selectedColor != 0: # change back to != 0
            if DAS_started is False:
                mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component, MAV_CMD_DO_SET_SERVO, 0, 1 , selectedColor , 0 , 0, 0, 0, 0) 
                DAS_started = True
                
            ret, frame = cap.read()  
            
            results = model.predict(frame, conf=0.4)

            # print("Color Code: ------ >>>  ", color_class)
            
            ### Lets change this part
            """if selectedColor != 0: 
            can be used to allow the model to be running but we arnt grabbing GPS calculations
            this will make it so we can wait until we are about to fly over targets to get target data to eliminate error and homeless trash false positives
            """
        
            for r in results :
                if(r.boxes.cls is not None and len(r.boxes.cls) > 0) :
                    rotate_image_array  = rotate_image( frame, telem_info[2] )
                    for i, class_label in enumerate(r.boxes.cls):
                        class_label_value = class_label.item()
                # Transforming the class data from a tensor to a numpy array
                class_data = r.boxes.cls.cpu().numpy()
                # print("\nclass data is:  -------- \n", class_data)
                index_of_target = np_where(class_data == color_class) # #### Target filter ####
                # print("\ntarget is class:  -------- ", color_class)
                index_of_target = tuple_to_int(index_of_target)
                # print("\nindex of target is:  ====== ", index_of_target)
                box_data = r.boxes.xywh.cpu().numpy()

                ##### GPS CALCULATIONS #####
                if box_data[index_of_target].size != 0:
                    if detected == False:
                        detected = True
                        mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component, MAV_CMD_DO_SET_SERVO, 0, 1 , detectedValue , 0 , 0, 0, 0, 0) # it saw a target Tell GS
                        sleep(.5)
                        # print("=== hit on target ====\n")
                        
                    # lat, lon, hdg, pitch, roll, alt
                    target_center = box_data[index_of_target][0][0:2]
                    dist_x, dist_y = find_pixel_dist(target_center, rotate_image_array)
                    target_dimensions = box_data[index_of_target][0][2:4]
                    dist_x_ft, dist_y_ft = pix_to_feet(dist_x, dist_y, target_dimensions[0],telem_info[3], telem_info[4], telem_info[5])
                    lat, lon = ft_to_latlong_to_gps(  (telem_info[0], telem_info[1]) , dist_x_ft, dist_y_ft, lon_conversion, lat_conversion ) 
                    coordinates[color_class].append((lat, lon))
    
    # /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            # ######## REMOVE THIS TO SEE REAL FPS (This will show the live feed) ################
            # gps_on_video = str(telem_info[0]) + " , " + str(telem_info[1])
            # putText(frame, gps_on_video, (50, 50), FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, LINE_AA)            
            # frame_ = results[0].plot()
            # frame_ = imutils_resize(frame_, width=1080) # width of my screen is 1536  
            # imshow('frame', frame_)
            # if waitKey(1) & 0xFF == ord('q'):  ##### COMMENT OUT ######
            #     break
            # print("FPS: ", 1.0 / (time() - start_time)) # FPS = 1 / time to process loop
            # # END OF fps CALCULATION
            # ## commewnt out to here #### 
            
    # ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        else:
            mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component, MAV_CMD_DO_SET_SERVO, 0, 1 , 1600 , 0 , 0, 0, 0, 0) if colorSelectedMsg is False else ()
            colorSelectedMsg = True if colorSelectedMsg is False else ()
            print("Waiting for color selection") 
            sleep(2)  



    cap.release() 
# ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    # destroyAllWindows()  
#/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    del model
    return_coords(color_class , coordinates)
    
 
 
 
 
def sendMission (coordinates):
    mav_conn.waypoint_clear_all_send()
    sleep(2)  # Wait for the clear to complete

    lat = coordinates[0]
    lon = coordinates[1]
    # Define the waypoint
    waypoint = mavutil.mavlink.MAVLink_mission_item_message(
        mav_conn.target_system,
        mav_conn.target_component,
        0,  # Sequence number
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Global frame, relative altitude
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # The command for navigating to a waypoint
        1,  # Current waypoint - set to true to make it active
        1,  # Autocontinue - set to true
        0, 0, 0, 0,  # Params 1-4 are not used
        lat, lon, 0  # Latitude, Longitude, Altitude
    )

    # Send waypoint count (1 in this case)
    mav_conn.waypoint_count_send(1)

    # Wait for the vehicle to request the waypoint
    request = mav_conn.recv_match(type='MISSION_REQUEST', blocking=True)
    print("Vehicle requested waypoint", request.seq)

    # Send the defined waypoint
    mav_conn.mav.send(waypoint)
    print(f"Sending waypoint: Lat {lat}, Lon {lon}, Alt {0}")

    # Wait for mission acknowledgment
    ack = mav_conn.recv_match(type='MISSION_ACK', blocking=True)
    if ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
        print("Mission accepted by vehicle.")
    else:
        print("Mission upload failed with error:", ack.type)
     
# ** Function that will calculate the average for coordinates and pass it to be sent to ground ** #
def return_coords(target , coordinates):
    global detected
    if detected == True:
        np_set_printoptions(precision=16)
        
        lat_values = [tuple[0] for tuple in coordinates[target]]
        lon_values = [tuple[1] for tuple in coordinates[target]]

        numpy_lat_values = np_array(lat_values)
        numpy_lon_values = np_array(lon_values)

        average_latitude = np_mean(numpy_lat_values)
        average_longitude = np_mean(numpy_lon_values)
        final_coordinates = (average_latitude , average_longitude)
        
        sendMission(final_coordinates)
        sleep(1)
        mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component, MAV_CMD_DO_SET_SERVO, 0, 1, 1440, 0, 0, 0, 0, 0)
    else:
        mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component, MAV_CMD_DO_SET_SERVO, 0, 1, 800, 0, 0, 0, 0, 0) # tell GS did not detect target
        print("=== no detection case ====\n")
        sleep(1)

    restartJetson()

def restartJetson():
    
    restart = False
    while True:
        SERVO_INFO = mav_conn.recv_match(type="SERVO_OUTPUT_RAW", blocking=True)
        if SERVO_INFO.servo4_raw == 1900:
            restart = True
            # Send MAV_CMD_DO_SET_SERVO command to set servo4_raw to 0
            mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component, MAV_CMD_DO_SET_SERVO, 0, 4, 0, 0, 0, 0, 0, 0)
            break
        print("Waiting for restart command")
        sleep(1) ## can we lower this

    if restart:
        mav_conn.close()
        print("Restarting the program")
       
        python = executable
        execl(python, python, *argv)

       
def request_message_interval(message_id, frequency_hz):
    mav_conn.mav.command_long_send(
    mav_conn.target_system, mav_conn.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
    message_id, 1000000 // frequency_hz, 0, 0, 0, 0, 0)
    
    
def update_telem_info(telem_info):
    
    global stopDetection
    global selectedColor
    global DAS_started
    global color_class
    global detected
    global detectedValue
    detectedValue = 898
    
    # lat, lon, hdg, pitch, roll, alt
    
    mav_conn = mavlink_connection(flightControllerString, baud=57600)
    
    # Continuously update the telem_info list with the latest GPS data
    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 10)  # Every 1 second for raw GPS data     
    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10)  # Every 1 second for raw GPS data                   
    while True:
        msg = mav_conn.recv_match(type=['GPS_RAW_INT'], blocking=True)
        msg2 = mav_conn.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True)
        msg3 = mav_conn.recv_match(type=['ATTITUDE'], blocking=True)
        if msg:
            if msg.get_type() == 'GPS_RAW_INT':
                telem_info[0] =  msg.lat / 1e7
                telem_info[1] = msg.lon / 1e7
                # print("lat----",telem_info[0],"lon----", telem_info[1],"alt----", telem_info[2])
        if msg2:
            if msg2.get_type() == 'GLOBAL_POSITION_INT':
                telem_info[2] = msg2.hdg / 100 # heading
                # telem_info[3] = msg2.pitch / 100.0
                # telem_info[4] = msg2.roll / 100.0
                telem_info[5] = msg2.relative_alt / 1000.0
                # print("alt----",telem_info[5])
        if msg3:
            if msg3.get_type() == 'ATTITUDE':
                telem_info[3] = msg3.pitch
                telem_info[4] = msg3.roll
        # lat, lon, alt = gps_reader.get_gps_data()
        
        # For resetting the color 
        # print('New location of stopping detectiona and resetting the color')
        SERVO_INFO = mav_conn.recv_match(type="SERVO_OUTPUT_RAW",blocking=True)
        servo3 = SERVO_INFO.servo3_raw
        # servo4 = SERVO_INFO.servo4_raw
        servo8 = SERVO_INFO.servo8_raw
        
        if servo8 != selectedColor:
            detectedValue += 2
            DAS_started = False
            selectedColor = servo8
            color_class = COLORS_PWM.index(selectedColor)
            detected = False
            
            
        if servo3 == 1000:
            print("Stopping the yolo")
            stopDetection = True
            break    

 
        
# ** Function that will start the multithreading and setup the detection and calculation ** #
if __name__=="__main__":
    
    init_result_event = Event()
    initial_checkup = Thread(target=initial_checkup, args=(init_result_event,))
    initial_checkup.start()
    initial_checkup.join()
    
    # Start the flight data logging function in a separate thread
    data_logger_thread = Thread(target=perform_file_operations, args=())
    data_logger_thread.start()
    data_logger_thread.join()
    
    # mav_conn = mavutil.mavlink_connection(flightControllerString, baud=57600)
    if init_result_event.is_set():
        ready = False
        mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component, MAV_CMD_DO_SET_SERVO, 0, 5 , 1500 , 0 , 0, 0, 0, 0)
        msg = mav_conn.recv_match(type = ['COMMAND_ACK'],blocking = True, timeout = 10)
        while True:
            print("WAITING.............")
            msg = mav_conn.recv_match(type='SERVO_OUTPUT_RAW',blocking=True)
            if msg.servo2_raw == 1200:
               ready = True
               ### should 1500 be changed to zero here????????? ######
               break
        if ready:
            mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component, MAV_CMD_DO_SET_SERVO, 0, 1 , 1900 , 0 , 0, 0, 0, 0)
            msg = mav_conn.recv_match(type = ['COMMAND_ACK'],blocking = True, timeout = 1)
            sleep(1)
            print("Initialization was successful starting the program")
            # mav_conn.close()
            # Updating the telem_info with the latest GPS data from gps_reader.py
            # Start the telem_info update function in a separate thread
            telem_info_updater_thread = Thread(target=update_telem_info, args=(telem_info,))
            telem_info_updater_thread.start()
                    
            # flight_data = Thread( target=get_heartbeat, args= (telem_info,) )
            obj_det = Thread( target=frame_retrival_and_detection, args= (coordinates,) )
            # flight_data.start()
            obj_det.start()
            
           
    else:
        mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component, MAV_CMD_DO_SET_SERVO, 0, 1 , 1950 , 0 , 0, 0, 0, 0)
        msg = mav_conn.recv_match(type='COMMAND_ACK' ,blocking=True)
        print("Initialization was not successful, please check the connections and try again")
        print("Exiting the program")
        # mav_conn.close()
        exit()  
        
################# Code Tables for Mavlink Commands #################################################################
# Documentation for all the servo pwm codes
# SERVO 1 - JETSON to GROUND
# _________________________________________________
# 1500 --> jetson ready 
# 1900 --> cam, gpu and telem ready
# 1600 --> waiting for the color ...
# selectedColor  to know which color was sent as well as that the DAS is working now 

# 1440 DAS stopped
# 1200 --> cam failed
# 1250 --> gpu failed
# 1850 --> telem connection failed (usb connection)
# 1870 --> telem connection failed (serial connection) THESE TWO WONT RUN IF WE HAVE CONNECTION PROBLEM WTF I WAS THINKING ????
# 1950 --> all failed  

# on getheartbeat function made changes : \
#  switcher = {
            #     1750: 0,
            #     1650: 1,
            #     1450: 2,
            #     1550: 3,
            #     1850: 4
            # }
        
        
        
        
        
               
# ** Function that will communicate with the flight controller and recieve telemetart info ** #
# def get_heartbeat(telem_info):
    
#     global stopDetection
#     global selectedColor
#     global DAS_started
#     global color_class
#     global detected
    
#     # mav_conn = mavutil.mavlink_connection(flightControllerString, baud=57600)
    
#     while True:
   
#         # print("=============================================================")  
#         PA_INFO = mav_conn.recv_match(type="GLOBAL_POSITION_INT",blocking=True)
#         telem_info[0] = PA_INFO.lat/10e6
#         telem_info[1] = PA_INFO.lon/10e6
#         telem_info[2] = PA_INFO.hdg/100
#         # print(PA_INFO)
#         # print("lat",telem_info[0],"lon", telem_info[1],"hdg", telem_info[2])
#         # print("=============================================================")
#         SERVO_INFO = mav_conn.recv_match(type="SERVO_OUTPUT_RAW",blocking=True)
#         servo3 = SERVO_INFO.servo3_raw
#         # servo4 = SERVO_INFO.servo4_raw
#         servo8 = SERVO_INFO.servo8_raw
        
#         if servo8 != selectedColor:
#             DAS_started = False
#             selectedColor = servo8
#             color_class = COLORS_PWM.index(selectedColor)
#             detected = False
            
            
#         if servo3 == 1000:
#             print("Stopping the yolo")
#             stopDetection = True
#             break    

# def flight_data_logger():
#     data_logger.perform_file_operations() 


# try:
#     # Specify the paths to your files
#     source_file_path = '/home/aero/Desktop/YOLOv8/Yolov8_organized/tensorRT_model/Multithreading/mav_waypoints.txt'
#     destination_file_path = '/home/aero/Desktop/YOLOv8/Yolov8_organized/tensorRT_model/Multithreading/waypoint_log.txt'

#     # Step 1: Read the contents from the source file
#     with open(source_file_path, 'r') as source_file:
#         contents = source_file.read()

#     # Step 2: Append the contents to the destination file
#     with open(destination_file_path, 'a') as destination_file:
#         destination_file.write("\n")
#         # Write date and time to the file   
#         destination_file.write(strftime("----------------%Y-%m-%d %H:%M:%S ----------------\n", localtime()))
#         destination_file.write(contents)

#     # Step 3: Erase the contents of the original file by opening it in write mode
#     with open(source_file_path, 'w'):
#         pass  # Just opening and closing the file is enough to erase its contents


#     mav_conn = mavlink_connection(flightControllerString, baud=57600)
#     mav_conn.wait_heartbeat(timeout=30)
#     print("Mavlink connection established and Heartbeat recieved (Timed out after 30 seconds))")
    
#     #mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component,mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 5 , 0 , 0 , 0, 0, 0, 0)
#     sleep(3)

# except Exception as e:
#     # print(f"Error creating Mavlink connection and/or recieving the heartbeat: {str(e)}")
#     print("we fucked up (usb from orange cube or some other connection error to fc)")
#     # mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component,mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 1 , 1870 , 0 , 0, 0, 0, 0)
#     exit()