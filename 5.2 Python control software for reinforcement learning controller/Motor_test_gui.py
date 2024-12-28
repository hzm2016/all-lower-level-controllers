'''
This GUI is developed for motor control 
'''
import serial
from serial.tools import list_ports
import struct
import time
import datetime as dt
from math import *
import csv
from PyQt5 import QtWidgets, QtCore, QtGui
from pyqtgraph import PlotWidget, plot 
import pyqtgraph as pg  
from PyQt5.QtWidgets import *  
from PyQt5.QtGui import * 
from PyQt5.QtCore import * 
import sys  
import json  
import numpy as np  

# import threading 
#thread = threading.Thread(target=read_from_serial)
#thread.daemon = True
#thread.start()

def find_available_ports():
    connected_ports = []

    # Get a list of all available ports
    available_ports = list(list_ports.comports())

    for port, desc, hwid in available_ports:
        try:
            # Attempt to open the serial port
            ser = serial.Serial(port)
            
            # Check if there is something connected to the port
            if ser.readable():
                connected_ports.append(port)
                
            # Close the serial port
            ser.close()
        except serial.SerialException:
            pass

    return connected_ports

win_size  = 150 # Quantity of data points to be displayed in the GUI when real-time plotting
t_buffer                = list([0] * win_size)
L_IMU_buffer            = t_buffer.copy()
R_IMU_buffer            = t_buffer.copy()
BattVolt_buffer         = t_buffer.copy()
L_motor_torque_buffer   = t_buffer.copy()
R_motor_torque_buffer   = t_buffer.copy()
L_motor_torque_d_buffer = t_buffer.copy()
R_motor_torque_d_buffer = t_buffer.copy()
L_motor_angpos_buffer   = t_buffer.copy()  
R_motor_angpos_buffer   = t_buffer.copy()  

L_motor_angpos_a_buffer   = t_buffer.copy()  
R_motor_angpos_a_buffer   = t_buffer.copy()  

L_leg_IMU_angle = 0
R_leg_IMU_angle = 0
L_motor_torque = 0
R_motor_torque = 0
L_motor_torque_desired = 0
R_motor_torque_desired = 0
L_motor_angpos = 0
R_motor_angpos = 0  
L_motor_angpos_a = 0  
R_motor_angpos_a = 0  

red  = pg.mkPen(color=(255, 0, 0), width = 2)
blue = pg.mkPen(color=(0, 0, 255), width = 2)

Connection_Flag    = False
LogginButton_Flag  = False
Transfer_Flag      = False
Data_Received_Flag = False
first_teensy_time  = True

t = 0
t_teensy = 0
t_minus_1 = 0
t_0   = 0
tau_d = 0
q_d   = 0

value_scale_send = 10.0     
value_scale_receive = 100.0     

class MainWindow(QWidget):  

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        global t_buffer, L_IMU_buffer, R_IMU_buffer, BattVolt_buffer, L_motor_torque_buffer, R_motor_torque_buffer,\
            L_motor_torque_d_buffer, R_motor_torque_d_buffer, L_motor_angpos_buffer, R_motor_angpos_buffer, \
            L_motor_angpos_a_buffer, R_motor_angpos_a_buffer, \
            t_0, t_minus_1,\
            tau_d, q_d,\
            ConnectButton, CmdButton, LoggingButton, Cmd_text, SerialComboBox,\
            Connection_Flag, connected_ports, Stiffness_text, Damping_text, \
            Stiffness_gain_box, Damping_gain_box, FF_force_gain_box, Assist_ratio_box, ref_pos_ampl_box, ref_pos_fre_box, ref_force_ampl_box, ref_force_fre_box  
           
        self.setWindowTitle('Motor Impedance Control Test')   
        # self.setWindowTitle('Motor Test')   
        self.setWindowIcon(QtGui.QIcon('BIRO_logo.png'))   
        
        connected_ports = find_available_ports()

        # Layout definition
        MainLayout    = QHBoxLayout() # Window Layout
        LP_Layout     = QVBoxLayout() # Left Pannel Layout
        RTDD_Layout   = QHBoxLayout() # Real-Time Data Display Layout
        Comm_Layout   = QHBoxLayout() # Layout for the COM pot selection and the Log data button
        Cmd_Layout    = QHBoxLayout() # Layout for sending commands
        LMotorLayout  = QVBoxLayout()
        RMotorLayout  = QVBoxLayout()  

        # Set the Main window layout
        self.setLayout(MainLayout)

        # Adding the sublayouts to the Main Layout
        MainLayout.addLayout(LP_Layout)
        MainLayout.addLayout(RTDD_Layout, stretch=5)

        # Adding the subsublayouts
        LP_Layout.addLayout(Comm_Layout)    
        LP_Layout.addLayout(Cmd_Layout)   
        
        RTDD_Layout.addLayout(LMotorLayout, stretch=5)
        RTDD_Layout.addLayout(RMotorLayout, stretch=5)   
        
        # LP_Layout.addLayout(Ref_Layout, stretch=5)    

        # Creating the Plot objects (Real-time data displays)
        LnR_IMU_plot        = pg.PlotWidget() # Real-time data display for the IMUs
        L_Motor_TnTd_plot   = pg.PlotWidget()
        L_Motor_AngPos_plot = pg.PlotWidget()
        R_Motor_TnTd_plot   = pg.PlotWidget()
        R_Motor_AngPos_plot = pg.PlotWidget()  

        ##############################
        ##### Left Pannel Layout #####
        ##############################

        # Ctrls_Layout objects (creation & arrangement)
        ## Objects
        ConnectButton  = QPushButton("Connect")
        SerialComboBox = QComboBox()
        LoggingButton  = QPushButton("Data Logging")
        ## Objects Functions
        ConnectButton.clicked.connect(Connect_Clicked)
        LoggingButton.clicked.connect(LogginButton_Clicked)  
        ## Arrangement
        Comm_Layout.addWidget(QLabel("ComPort:"))  
        Comm_Layout.addWidget(SerialComboBox)
        SerialComboBox.addItems(connected_ports) 
        Comm_Layout.addWidget(ConnectButton)
        Comm_Layout.addWidget(LoggingButton)  
        
        ##############################################
        ## Parameter Adjustment ##
        ##############################################
        Param_block = QGroupBox("Parameter Adjustment")    
        Param_block_Layout = QGridLayout()  
        Param_block.setLayout(Param_block_Layout)   
        Cmd_Layout.addWidget(Param_block)   
        # Parameters selectors
        Stiffness_gain_box = QDoubleSpinBox()   
        Damping_gain_box   = QDoubleSpinBox()   
        FF_force_gain_box  = QDoubleSpinBox()   
        Assist_ratio_box   = QDoubleSpinBox()   

        Custom_QDoubleSpinBox(Stiffness_gain_box, value=0.0, minimum=0.0, maximum=100.0, single_step=0.2, decimals=2, prefix="", suffix="")   
        Custom_QDoubleSpinBox(Damping_gain_box, value=0.0, minimum=0.0, maximum=5.0, single_step=0.2, decimals=2, prefix="", suffix="")    
        Custom_QDoubleSpinBox(FF_force_gain_box, value=0.0, minimum=0.0, maximum=10.0, single_step=0.2, decimals=2, prefix="", suffix="")    
        Custom_QDoubleSpinBox(Assist_ratio_box, value=0.0, minimum=0, maximum=1.0, single_step=0.1, decimals=2, prefix="", suffix="")    

        Stiffness_gain_box.setValue(1)
        Damping_gain_box.setValue(0)
        FF_force_gain_box.setValue(0)
        Assist_ratio_box.setValue(0.1)   

        # set paper
        Stiffness_gain_box.valueChanged.connect(Send_Parameters)
        Damping_gain_box.valueChanged.connect(Send_Parameters)
        FF_force_gain_box.valueChanged.connect(Send_Parameters)
        Assist_ratio_box.valueChanged.connect(Send_Parameters)

        # Stiffness_gain_box_lb = QLabel("Stiffness [Nm/deg]")      
        # Damping_gain_box_lb   = QLabel("Damping [Nm/deg/s]")      
        # FF_force_gain_box_lb  = QLabel("Feedforward Force [Nm]")       
        # Assist_ratio_box_lb   = QLabel("Assistive Scale [0, 1]")       
        
        Stiffness_gain_box_lb = QLabel("Stiffness [Nm/rad]")      
        Damping_gain_box_lb   = QLabel("Damping [Nm*s/rad]")      
        FF_force_gain_box_lb  = QLabel("Feedforward Force [Nm]")         
        Assist_ratio_box_lb   = QLabel("Assistive Scale [0, 1]")       
        
        sw_label_row = 1   
        sw_ctrls_row = sw_label_row + 1   
        Param_block_Layout.addWidget(Stiffness_gain_box_lb, sw_label_row, 1)
        Param_block_Layout.addWidget(Damping_gain_box_lb, sw_label_row, 2)
        Param_block_Layout.addWidget(FF_force_gain_box_lb, sw_label_row, 3)
        Param_block_Layout.addWidget(Assist_ratio_box_lb, sw_label_row, 4)
        Param_block_Layout.addWidget(Stiffness_gain_box, sw_ctrls_row, 1)
        Param_block_Layout.addWidget(Damping_gain_box, sw_ctrls_row, 2)
        Param_block_Layout.addWidget(FF_force_gain_box, sw_ctrls_row, 3)   
        Param_block_Layout.addWidget(Assist_ratio_box, sw_ctrls_row, 4)  
        
        # # reference force or position 
        # Ref_param_block = QGroupBox("Reference Adjustment")    
        # Ref_Param_block_Layout = QGridLayout()  
        # Ref_param_block.setLayout(Ref_Param_block_Layout)   
        # Ref_Layout.addWidget(Ref_param_block)   
        
        # Parameters selectors
        ref_pos_ampl_box = QDoubleSpinBox()
        ref_pos_fre_box   = QDoubleSpinBox()
        ref_force_ampl_box = QDoubleSpinBox()
        ref_force_fre_box   = QDoubleSpinBox()
    
        Custom_QDoubleSpinBox(ref_pos_ampl_box, value=0.0, minimum=0.0, maximum=5.0, single_step=0.2, decimals=2, prefix="", suffix="")   
        Custom_QDoubleSpinBox(ref_pos_fre_box, value=0.0, minimum=-20.0, maximum=20.0, single_step=0.2, decimals=2, prefix="", suffix="")    
        Custom_QDoubleSpinBox(ref_force_ampl_box, value=0.0, minimum=-20.0, maximum=20.0, single_step=0.2, decimals=2, prefix="", suffix="")    
        Custom_QDoubleSpinBox(ref_force_fre_box, value=0.0, minimum=0.0, maximum=10.0, single_step=0.1, decimals=2, prefix="", suffix="")    

        ref_pos_ampl_box.setValue(0.2)   
        ref_pos_fre_box.setValue(1)    
        ref_force_ampl_box.setValue(1)     
        ref_force_fre_box.setValue(1)    

        # set paper
        ref_pos_ampl_box.valueChanged.connect(Send_Parameters)
        ref_pos_fre_box.valueChanged.connect(Send_Parameters)
        ref_force_ampl_box.valueChanged.connect(Send_Parameters)
        ref_force_fre_box.valueChanged.connect(Send_Parameters)

        ref_pos_ampl_box_lb    = QLabel("Position [rad]")      
        ref_pos_fre_box_lb     = QLabel("Position Frequency [Hz]")      
        ref_force_ampl_box_lb  = QLabel("Force [Nm]")       
        ref_force_fre_box_lb   = QLabel("Force Frequency [Hz]")       
        
        sw_label_row = 3  
        sw_ctrls_row = sw_label_row + 1   
        Param_block_Layout.addWidget(ref_pos_ampl_box_lb, sw_label_row, 1)
        Param_block_Layout.addWidget(ref_pos_fre_box_lb, sw_label_row, 2)
        Param_block_Layout.addWidget(ref_force_ampl_box_lb, sw_label_row, 3)
        Param_block_Layout.addWidget(ref_force_fre_box_lb, sw_label_row, 4)
        Param_block_Layout.addWidget(ref_pos_ampl_box, sw_ctrls_row, 1)
        Param_block_Layout.addWidget(ref_pos_fre_box, sw_ctrls_row, 2)
        Param_block_Layout.addWidget(ref_force_ampl_box, sw_ctrls_row, 3)    
        Param_block_Layout.addWidget(ref_force_fre_box, sw_ctrls_row, 4)   
        
        #########################################
        ##### Real-Time Data Display Layout #####
        #########################################
        # Adding the IMUs plot at the bottom of the Left Pannel Layout
        LP_Layout.addWidget(LnR_IMU_plot)

        # Left MotorLayout
        LMotorLayout.addWidget(L_Motor_TnTd_plot)  
        LMotorLayout.addWidget(L_Motor_AngPos_plot)   

        # Rigth MotorLayout  
        RMotorLayout.addWidget(R_Motor_TnTd_plot)  
        RMotorLayout.addWidget(R_Motor_AngPos_plot)  

        #############################################
        ##### Configuring the look of the plots #####
        #############################################
               
        # General style
        label_style = {"color": "black", "font-size": "18px"}
        title_style = {"color": "black", "font-size": "26px"}

        # IMUs Plot  
        LnR_IMU_plot.setTitle("Thighs angular position", **title_style)
        LnR_IMU_plot.setLabel('left', "Angle [deg]", **label_style)
        LnR_IMU_plot.setLabel('bottom', "Time [s]", **label_style)
        LnR_IMU_plot.addLegend()
        LnR_IMU_plot.setBackground('w')
        LnR_IMU_plot.showGrid(x=True, y=True)  
        self.L_IMU_line = LnR_IMU_plot.plot(t_buffer, L_IMU_buffer, name = "Left", pen = red)
        self.R_IMU_line = LnR_IMU_plot.plot(t_buffer, R_IMU_buffer, name = "Right", pen = blue)

        ## Left motor ##
        # Torque
        L_Motor_TnTd_plot.setTitle("Actuator 1 Torque (Left)", **label_style)  
        L_Motor_TnTd_plot.setLabel('left', "Torque [Nm]", **label_style) 
        L_Motor_TnTd_plot.setLabel('bottom', "Time [s]", **label_style) 
        L_Motor_TnTd_plot.addLegend() 
        L_Motor_TnTd_plot.setBackground('w')  
        L_Motor_TnTd_plot.showGrid(x=True, y=True)    
        self.L_Motor_Taud_line = L_Motor_TnTd_plot.plot(t_buffer, L_motor_torque_d_buffer, name = "Reference Torque", pen = blue)
        self.L_Motor_Tau_line  = L_Motor_TnTd_plot.plot(t_buffer, L_motor_torque_buffer, name = "Actual Torque", pen = red)
        # Position  
        L_Motor_AngPos_plot.setTitle("Actuator 1 Position (Left)", **title_style)
        L_Motor_AngPos_plot.setLabel('left', "Ang. Position [rad]", **label_style)
        L_Motor_AngPos_plot.setLabel('bottom', "Time [s]", **label_style) 
        L_Motor_AngPos_plot.addLegend()  
        L_Motor_AngPos_plot.setBackground('w')   
        L_Motor_AngPos_plot.showGrid(x=True, y=True)   
        self.L_motor_angpos_line = L_Motor_AngPos_plot.plot(t_buffer, L_motor_angpos_buffer, name="Reference Position", pen = red)  
        self.L_motor_angpos_a_line = L_Motor_AngPos_plot.plot(t_buffer, L_motor_angpos_a_buffer, name="Actual Position", pen = blue)     

        ## Right motor ##
        # Torque
        R_Motor_TnTd_plot.setTitle("Actuator 2 Torque (Right)", **title_style)
        R_Motor_TnTd_plot.setLabel('left', "Torque [Nm]", **label_style)
        R_Motor_TnTd_plot.setLabel('bottom', "Time [s]", **label_style)
        R_Motor_TnTd_plot.addLegend()
        R_Motor_TnTd_plot.setBackground('w')   
        R_Motor_TnTd_plot.showGrid(x=True, y=True)
        self.R_Motor_Taud_line = R_Motor_TnTd_plot.plot(t_buffer, R_motor_torque_d_buffer, name = "Reference Torque", pen = blue)
        self.R_Motor_Tau_line  = R_Motor_TnTd_plot.plot(t_buffer, R_motor_torque_buffer, name = "Actual Torque", pen = red)
        # Position
        R_Motor_AngPos_plot.setTitle("Actuator 2 Position (Right)", **title_style)
        R_Motor_AngPos_plot.setLabel('left', "Ang. Position [rad]", **label_style)
        R_Motor_AngPos_plot.setLabel('bottom', "Time [s]", **label_style)  
        R_Motor_AngPos_plot.addLegend()     
        R_Motor_AngPos_plot.setBackground('w')    
        R_Motor_AngPos_plot.showGrid(x=True, y=True)    
        self.R_motor_angpos_line = R_Motor_AngPos_plot.plot(t_buffer, R_motor_angpos_buffer, name="Reference Position", pen = red)   
        self.R_motor_angpos_a_line = R_Motor_AngPos_plot.plot(t_buffer, R_motor_angpos_a_buffer, name="Actual Position", pen = blue)   

        # Creation of the timer for executing the function repetitively
        self.timer = QtCore.QTimer()   
        self.timer.setInterval(20) # Set the refresh time-rate for the plotted data in the GUI (every x miliseconds)
        self.timer.timeout.connect(self.all) # This function is called 20 times per second [50Hz] (Fastests stable)
        self.timer.start() 


    def all(self):  
        global Connection_Flag, Data_Received_Flag
        
        if Connection_Flag:
            if ser.in_waiting > 0:
                ConnectButton.setText("Receiving")
                ConnectButton.setStyleSheet("background-color : green")
                Recieve_data()
                if Data_Received_Flag: 
                    self.update_plot_data()  
                    Data_Received_Flag = False  
                
   
    def update_plot_data(self):
        global t_buffer, L_IMU_buffer, R_IMU_buffer, BattVolt_buffer, L_motor_torque_buffer, R_motor_torque_buffer,\
            L_motor_torque_d_buffer, R_motor_torque_d_buffer, L_motor_angpos_buffer, R_motor_angpos_buffer, L_motor_angpos_a_buffer, R_motor_angpos_a_buffer, \
            L_leg_IMU_angle, R_leg_IMU_angle, L_motor_torque, R_motor_torque,\
            L_motor_torque_desired, R_motor_torque_desired, t_teensy, L_motor_angpos, R_motor_angpos,\
            t_0, t_minus_1,\
            tau_d, q_d,\
            Connection_Flag, LogginButton_Flag,\
            csv_file_name, DataHeaders, t_0_teensy

        if Connection_Flag == True:

            t = time.time() - t_0

            if t_minus_1 != t:  
                t_buffer = t_buffer[1:]
                t_buffer.append(t_teensy - t_0_teensy)

                L_IMU_buffer = L_IMU_buffer[1:]
                L_IMU_buffer.append(L_leg_IMU_angle)

                R_IMU_buffer = R_IMU_buffer[1:]
                R_IMU_buffer.append(R_leg_IMU_angle)

                BattVolt_buffer = BattVolt_buffer[1:]   
                BattVolt_buffer.append(t_teensy)  

                # /////// left   
                L_motor_torque_d_buffer = L_motor_torque_d_buffer[1:]
                L_motor_torque_d_buffer.append(L_motor_torque_desired)

                L_motor_torque_buffer = L_motor_torque_buffer[1:]
                L_motor_torque_buffer.append(L_motor_torque)

                L_motor_angpos_buffer = L_motor_angpos_buffer[1:]    
                L_motor_angpos_buffer.append(L_motor_angpos)    

                L_motor_angpos_a_buffer = L_motor_angpos_a_buffer[1:]   
                L_motor_angpos_a_buffer.append(L_motor_angpos_a)     

                # /////// right 
                R_motor_torque_d_buffer = R_motor_torque_d_buffer[1:]  
                R_motor_torque_d_buffer.append(R_motor_torque_desired)

                R_motor_torque_buffer = R_motor_torque_buffer[1:]
                R_motor_torque_buffer.append(R_motor_torque)

                R_motor_angpos_buffer = R_motor_angpos_buffer[1:]
                R_motor_angpos_buffer.append(R_motor_angpos)   
                
                R_motor_angpos_a_buffer = R_motor_angpos_a_buffer[1:]   
                R_motor_angpos_a_buffer.append(R_motor_angpos_a)       
            
                # set data for each time 
                self.L_IMU_line.setData(t_buffer, L_IMU_buffer)  
                self.R_IMU_line.setData(t_buffer, R_IMU_buffer)   

                # set torque data 
                self.L_Motor_Taud_line.setData(t_buffer, L_motor_torque_d_buffer)
                self.L_Motor_Tau_line.setData(t_buffer, L_motor_torque_buffer)

                # set position data  
                self.L_motor_angpos_line.setData(t_buffer, L_motor_angpos_buffer)    
                self.L_motor_angpos_a_line.setData(t_buffer, L_motor_angpos_a_buffer)    

                # set torque error data  
                self.R_Motor_Taud_line.setData(t_buffer, R_motor_torque_d_buffer)  
                self.R_Motor_Tau_line.setData(t_buffer, R_motor_torque_buffer)  

                # set position error data
                self.R_motor_angpos_line.setData(t_buffer, R_motor_angpos_buffer)  
                self.R_motor_angpos_a_line.setData(t_buffer, R_motor_angpos_a_buffer)     

                if LogginButton_Flag == True:
                    LoggedData = {
                        "time": t,
                        "L_IMU": L_leg_IMU_angle,
                        "R_IMU": R_leg_IMU_angle,
                        "L_Torque_d": L_motor_torque_desired,
                        "L_Torque": L_motor_torque,
                        "L_AngPos": L_motor_angpos,
                        "R_Torque_d": R_motor_torque_desired,
                        "R_Torque": R_motor_torque,
                        "R_AngPos": R_motor_angpos
                    }

                    with open(csv_file_name, mode="a", newline="") as file:
                        writer = csv.DictWriter(file, fieldnames = DataHeaders)
                        writer.writerow(LoggedData)  
            t_minus_1 = t
        else:
            print("NOT Connected")  

 
def Connect_Clicked():  
    global ConnectButton, ser, Connection_Flag, t_0, ble_datalength, data_length, decoded_data, rs232_datalength 

    ### Defining the size of the received packages ###
    ble_datalength   = 32 # Recieved package data size
    rs232_datalength = 20 # Transmited package data size
    data_length      = ble_datalength - 3
    decoded_data     = [0]*data_length  
    ##################################################

    #### Setting up the serial communication port and baudrate ###
    serial_port = SerialComboBox.currentText()
    baud_rate   = 115200
    ############################################

    ### Stablish the serial connection ###
    ser = serial.Serial(port=serial_port, baudrate=baud_rate)
    ser.timeout = 0 # set read timeout
    ######################################

    while not ser.is_open:
        print('Serial port closed')
    
    if ser.is_open:  
        print('Serial port opened') 
        ConnectButton.setText("Bluetooth activated") 
        ConnectButton.setStyleSheet("background-color : yellow")  
        Connection_Flag = True  

        t_0 = time.time()         # Set the initial time    

def Recieve_data():   
    global ser, ble_datalength, data_length, decoded_data, Data_Received_Flag,\
            L_leg_IMU_angle, R_leg_IMU_angle, L_motor_torque, R_motor_torque,\
            L_motor_torque_desired, R_motor_torque_desired, t_teensy, L_motor_angpos, R_motor_angpos, L_motor_angpos_a, R_motor_angpos_a,\
            t_0_teensy, first_teensy_time
    
    if ser.in_waiting >= 32: 
        if ser.read(1) == b'\xA5':  # 165 in uint8
            if ser.read(1) == b'\x5A':  # 90 in uint8
                expected_length = ser.read(1)  # Read the length byte
                if expected_length == bytes([ble_datalength]):  # Check the length
                    if ser.in_waiting >= data_length:  # Ensure enough data is available
                        coded_data = ser.read(data_length)  
                        #decoded_data = [0] * (data_length // 2)  # Initialize decoded data array
                        decode_i = 0
                        for i in range(1, data_length, 2):
                            var = coded_data[i-1] + coded_data[i] * 256
                            var = (var - 65536)/value_scale_receive if var > 32767 else var/value_scale_receive
                            decoded_data[decode_i] = var   
                            decode_i += 1   
                    
                    L_leg_IMU_angle        = decoded_data[0]   
                    R_leg_IMU_angle        = decoded_data[1]  
                    L_motor_torque         = decoded_data[2]
                    R_motor_torque         = decoded_data[3]
                    L_motor_torque_desired = decoded_data[4]
                    R_motor_torque_desired = decoded_data[5]
                    t_teensy               = decoded_data[6]
                    L_motor_angpos         = decoded_data[7]   
                    L_motor_angpos_a       = decoded_data[8]   
                    R_motor_angpos         = decoded_data[9]   
                    R_motor_angpos_a       = decoded_data[10]       

                    Data_Received_Flag = True
                    if first_teensy_time:
                        t_0_teensy = t_teensy
                        first_teensy_time = False
    else:
        print('Waiting for the whole data package...')
        # ConnectButton.setText("Searching Hip Exoskeleton")
        # ConnectButton.setStyleSheet("background-color : orange")  
    
def CmdButton_Clicked():
    global Transfer_Flag,\
        Cmd_text, cmd, Stiffness_text, Damping_text, damping, stiffness  

    cmd = float(Cmd_text.text())    
    stiffness = float(Stiffness_text.text())   
    damping = float(Damping_text.text())     

    # if cmd < 0 or cmd > 2:
    #     showdialog()
    #     cmd = saturation(cmd, 0, 2)
    
    Transmit_data()
    print("| Command " + str(cmd) + " sent |")  

def Transmit_data():
    global rs232_datalength, data_package, ser, Stiffness_gain, Damping_gain, FF_force_gain, Assist_ratio_gain, ref_pos_gain, ref_pos_fre_gain, ref_force_ampl_gain, ref_force_fre_gain

    rs232_datalength       = 20   
    Stiffness_gain_byte    = pack_bytearray(Stiffness_gain)  
    Damping_gain_byte      = pack_bytearray(Damping_gain)
    FF_force_gain_byte     = pack_bytearray(FF_force_gain)  
    Assist_ratio_gain_byte = pack_bytearray(Assist_ratio_gain)    
    
    ref_pos_gain_byte        = pack_bytearray(ref_pos_gain)  
    ref_pos_fre_gain_byte    = pack_bytearray(ref_pos_fre_gain)  
    ref_force_ampl_gain_byte = pack_bytearray(ref_force_ampl_gain)   
    ref_force_fre_gain_byte  = pack_bytearray(ref_force_fre_gain)   
    
    print("Stiffness:, Damping:, FF_force, Assist_ratio_gain :", Stiffness_gain_byte[0], Damping_gain_byte[0], FF_force_gain_byte[0], Assist_ratio_gain_byte[0])   
    print("Ref_pos :{}, Ref_pos_fre :{}, Ref_force :{}, Ref_force_fre :{}", ref_pos_gain_byte[0], ref_pos_fre_gain_byte[0], ref_force_ampl_gain_byte[0], ref_force_fre_gain_byte[0])    
    
    # data_package = bytearray([165, 90, rs232_datalength, Stiffness_gain_byte[0], Stiffness_gain_byte[1], Damping_gain_byte[0], Damping_gain_byte[1], FF_force_gain_byte[0], FF_force_gain_byte[1], int(Assist_ratio_gain), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    data_package = bytearray([165, 90, rs232_datalength, 0, 0, 0, 0, 
                            int(Stiffness_gain_byte[0]), int(Damping_gain_byte[0]), int(FF_force_gain_byte[0]), int(Assist_ratio_gain_byte[0] * 10), 
                            int(ref_pos_gain_byte[0]), int(ref_pos_fre_gain_byte[0]), int(ref_force_ampl_gain_byte[0]), int(ref_force_fre_gain_byte[0]), 
                            0, 0, 0, 0, 0])    
    if ser.is_open:     
        ser.write(data_package)   
        # print(f"| {Stiffness_gain} | {Damping_gain} | {FF_force_gain} | {Assist_ratio_gain} |")

def LogginButton_Clicked():
    global LogginButton_Flag, LoggingButton, csv_file_name, DataHeaders, t_0
    LogginButton_Flag = True
    t_0 = time.time()
    LoggingButton.setText("Logging data") 
    LoggingButton.setStyleSheet("background-color : blue")
    csv_file_name = "GUI_Logger_" + time.strftime("%Y-%m-%d_%H-%M-%S") + ".csv"
    DataHeaders = ["time", "L_IMU", "R_IMU", "L_Torque_d", "L_Torque", "L_AngPos", "R_Torque_d", "R_Torque", "R_AngPos"]

    # Create the CSV file and write the header
    with open(csv_file_name, mode="w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames = DataHeaders)
        writer.writeheader()

def showdialog():
   msg = QMessageBox()
   msg.setIcon(QMessageBox.Warning)    
   msg.setWindowTitle("Warning")
   msg.setText("Value out of bounds")
   msg.setInformativeText("Please introduce a numerical value between 0 and 2.")
   msg.setDetailedText("Every value minor to 0 will be taken as 0 and every value greater that 2 will be taken as 2")
   msg.setStandardButtons(QMessageBox.Ok)
   msg.exec_()

def saturation(value, min_value, max_value):
    return max(min_value, min(value, max_value))

def Custom_QDoubleSpinBox(spinbox, value=0.0, minimum=0.0, maximum=100.0, single_step=1.0, decimals=2, prefix="", suffix=""):
    """
    Customizes the properties of a QDoubleSpinBox.

    Parameters:
    spinbox (QDoubleSpinBox): The spinbox to customize.
    value (float): Initial value of the spinbox.
    minimum (float): Minimum value of the spinbox.
    maximum (float): Maximum value of the spinbox.
    single_step (float): Step value for each increment/decrement.
    decimals (int): Number of decimal places.
    prefix (str): Text prefix displayed before the value.
    suffix (str): Text suffix displayed after the value.
    wrapping (bool): Whether the value wraps around at minimum/maximum.
    alignment (Qt.AlignmentFlag): Alignment of the text in the spinbox.
    """
    spinbox.setValue(value)
    spinbox.setRange(minimum, maximum)
    spinbox.setSingleStep(single_step)
    spinbox.setDecimals(decimals)
    spinbox.setPrefix(prefix)
    spinbox.setSuffix(suffix) 

def Send_Parameters(): 
    global Stiffness_gain_box, Damping_gain_box, FF_force_gain_box, Assist_ratio_box,\
            Stiffness_gain, Damping_gain, FF_force_gain, Assist_ratio_gain,\
            ref_pos_ampl_box, ref_pos_fre_box, ref_force_ampl_box, ref_force_fre_box,\
            ref_pos_gain, ref_pos_fre_gain, ref_force_ampl_gain, ref_force_fre_gain  
        
    Stiffness_gain    = Stiffness_gain_box.value()    
    Damping_gain      = Damping_gain_box.value()    
    FF_force_gain     = FF_force_gain_box.value()    
    Assist_ratio_gain = Assist_ratio_box.value()    
    
    ref_pos_gain = ref_pos_ampl_box.value()   
    ref_pos_fre_gain = ref_pos_fre_box.value()   
    ref_force_ampl_gain = ref_force_ampl_box.value()     
    ref_force_fre_gain = ref_force_fre_box.value()    

    Transmit_data()  

def pack_bytearray(value_to_pack):    
    bytearray = struct.pack('h', int(value_to_pack * value_scale_send))       
    return bytearray    

# def Transmit_data():   
#     global rs232_datalength, data_package, ser, Transfer_Flag,\
#           cmd, stiffness, damping 
    
#     value_scale = 100  

#     ##### assistive ratio ##### 
#     data_package = bytearray([165, 90, rs232_datalength, 0, 0, 0, 0, 0, 
#                               int(cmd*value_scale), int(stiffness*value_scale), int(damping*value_scale), 
#                               0, 0, 0, 0, 0, 0, 0, 0, 0])    
#     ##### second data  
#     if ser.is_open:   
#         ser.write(data_package)     

#     Transfer_Flag = False   


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    Window = MainWindow()   
    Window.show()  
    sys.exit(app.exec_())   