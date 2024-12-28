import sys
import time
import csv
import numpy as np
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QPushButton, QComboBox, QLabel, QGroupBox, QSlider, QMessageBox
from PyQt5.QtCore import Qt, QTimer
from serial import Serial
from serial.tools import list_ports

# Function to find available serial ports
def find_available_ports():
    ports = list(list_ports.comports())
    return [port.device for port in ports]

class MainWindow(QWidget):
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        self.setWindowTitle('Hip Exoskeleton Control Software v2.1')
        self.setMinimumSize(900, 600)
        self.overall_fontsize = 40

        # Initialize variables
        self.connected_ports = find_available_ports()
        self.t_0      = 0
        self.k_assist = 0 # Assistance factor
        self.Pk_T     = 0 # Peak torque
        self.LogginButton_Flag = False
        self.Calibration_Flag = 0
        self.Connection_Flag = 0
        self.DataHeaders = []

        # Layout definition
        MainLayout    = QVBoxLayout()  # Window Layout
        Top_Layout    = QHBoxLayout()  # Layout for the COM port selection and the Log data button

        # Initialize UI elements
        self.Create_DevicesCombobox()
        self.Create_ConnectButton()
        self.Create_LoggingButton()
        self.Create_Level_of_Assitance_Block()
        self.Create_StopButton()

        # Add widgets to layouts
        self.stretch_factor = 10
        self.Device_Label = QLabel("Device:")
        self.Device_Label.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        # Top_Layout.addWidget(QLabel("Device:"))
        Top_Layout.addWidget(self.Device_Label)
        Top_Layout.addWidget(self.DevicesCombobox, stretch=self.stretch_factor)
        Top_Layout.addWidget(self.ConnectButton, stretch=self.stretch_factor)
        Top_Layout.addWidget(self.LoggingButton, stretch=self.stretch_factor)

        MainLayout.addLayout(Top_Layout)
        MainLayout.addWidget(self.LVLoA_Block)
        MainLayout.addWidget(self.StopButton)
        self.setLayout(MainLayout)

        # Creation of the timer for executing the function repetitively
        # self.timer = QTimer()
        # self.timer.setInterval(10) # Set the refresh time-rate for the plotted data in the GUI (every x miliseconds)
        # self.timer.timeout.connect(self.All) # This function is called n times per second [n Hz]
        # self.timer.start()

    def All(self):
        # print("All()")
        if self.Connection_Flag:
            # print("All >>> Connected")
            # print(self.ser.is_open)
            # print(self.ser.in_waiting)
            #self.Receive_Exo_Data()
            if self.ser.in_waiting > 0:
                print("All >>> Connected >>> Receiving")
                self.ConnectButton.setText("Connected")
                self.ConnectButton.setStyleSheet("background-color : green")
                #self.Receive_Exo_Data()
                print("Here")
                # if self.Data_Received_Flag:
                #     self.Data_Received_Flag = False
    
    def Create_DevicesCombobox(self):
        self.DevicesCombobox = QComboBox()
        self.DevicesCombobox.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.DevicesCombobox.addItems(self.connected_ports)
    
    def Create_ConnectButton(self):
        self.ConnectButton = QPushButton("Connect")
        self.ConnectButton.setStyleSheet("background-color: rgb(150, 150, 150);"
                                    "color: white;"
                                    "font: Bold;"
                                    f"font-size: {self.overall_fontsize}px;")
        self.ConnectButton.clicked.connect(self.connect_device)

    def Create_LoggingButton(self):
        self.LoggingButton = QPushButton("Log Data")
        self.LoggingButton.setEnabled(False)
        self.LoggingButton.setStyleSheet("background-color: rgb(150, 150, 150);"
                                    "color: white;"
                                    "font: Bold;"
                                    f"font-size: {self.overall_fontsize}px;")
        self.LoggingButton.clicked.connect(self.loggin_button_clicked)

    def Create_Level_of_Assitance_Block(self):
        ## Level of Assistance Block
        self.LVLoA_Block  = QGroupBox("Level of Assistance")
        self.LVLoA_Block.setAlignment(Qt.AlignCenter)
        self.LVLoA_Block.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.LVLoA_Block_Layout = QGridLayout()
        self.LVLoA_Block.setLayout(self.LVLoA_Block_Layout)
        self.LVLoA_Block.setEnabled(False)
        self.k_assist_label = QLabel(f"{self.k_assist*10:.0f}%")
        self.k_assist_label.setAlignment(Qt.AlignCenter)
        # self.k_assist_label.setStyleSheet("font-size: 50px;")
        self.LVLoA_Block_Layout.addWidget(self.k_assist_label,1,0,1,11)
        ### Slider
        self.Slider = QSlider(Qt.Horizontal)
        self.LVLoA_Block_Layout.addWidget(QLabel("0"),2,0)
        self.LVLoA_Block_Layout.addWidget(QLabel("100"),2,10)
        self.LVLoA_Block_Layout.addWidget(self.Slider,3,0,3,11)
        self.Slider.setValue(0)
        self.Slider.setMinimum(0)
        self.Slider.setMaximum(10)        
        self.Slider.setTickPosition(QSlider.TicksBothSides)
        self.Slider.setTickInterval(1)
        self.Slider.setSingleStep(1)
        self.Pk_label = QLabel(f"Peak Torque: {self.Pk_T} Nm")
        self.Pk_label.setAlignment(Qt.AlignCenter)
        self.LVLoA_Block_Layout.addWidget(self.Pk_label,6,0,6,11)
        self.Slider.valueChanged.connect(self.Slider_ValueChange)
    
    def Create_StopButton(self):
        self.StopButton = QPushButton("STOP")
        self.StopButton.setEnabled(False)
        self.StopButton.setStyleSheet("background-color: rgb(150, 150, 150);"
                                      "color: white;"
                                      "font: Bold;"
                                      f"font-size: {2*self.overall_fontsize}px;")
        self.StopButton.clicked.connect(self.stop_button_clicked)

    # Function to connect to the selected device
    def connect_device(self):
        ### Defining the size of the received packages ###
        self.TXD_datapackage_length = 20 # Transmit data package size
        self.RXD_datapackage_length = 4 # Receive data package size
        self.data_length        = self.RXD_datapackage_length - 3
        self.decoded_data       = [0]*self.data_length
        self.BAUD_RATE          = 115200
        self.SER_PORT           = self.DevicesCombobox.currentText()

        self.ser = Serial(port = self.SER_PORT, baudrate = self.BAUD_RATE)
        self.ser.timeout = 0 # set read timeout
        if self.SER_PORT:
            print(f"Connecting to {self.SER_PORT}...")
            print(self.ser)
            if self.ser.is_open:
                print('Serial port opened')
                self.ConnectButton.setText("Connected")
                self.ConnectButton.setStyleSheet("background-color: green;"
                                                 "color: white;"
                                                 "font: Bold;"
                                                 f"font-size: {self.overall_fontsize}px;")
                
                self.StopButton.setEnabled(True)
                self.StopButton.setStyleSheet("background-color: rgb(255, 0, 0);"
                                              "color: white;"
                                              "font: Bold;"
                                              f"font-size: {2*self.overall_fontsize}px;")
                self.LoggingButton.setEnabled(True)
                self.LoggingButton.setStyleSheet("background-color: rgb(0, 0, 150);"
                                                 "color: white;"
                                                 "font: Bold;"
                                                 f"font-size: {self.overall_fontsize}px;")
                self.LVLoA_Block.setEnabled(True)
                self.Connection_Flag = True
                time.sleep(0.5)
                self.Encoder_Calibration_Msg()

    # Function to handle logging button click
    def loggin_button_clicked(self):
        if self.ser.is_open:
            self.LogginButton_Flag = True
            self.t_0 = time.time()
            self.LoggingButton.setText("Logging data")
            self.LoggingButton.setStyleSheet("background-color: rgb(0, 0, 255);"
                                        "color: white;"
                                        "font: Bold;"
                                        f"font-size: {self.overall_fontsize}px;")
            self.csv_file_name = f"GUI_Logger_{time.strftime('%Y-%m-%d_%H-%M-%S')}.csv"
            self.DataHeaders = ["time", "L_IMU", "R_IMU", "L_Torque_d", "L_Torque", "L_AngPos", "R_Torque_d", "R_Torque", "R_AngPos"]
            # Create the CSV file and write the header
            with open(self.csv_file_name, mode="w", newline="") as file:
                writer = csv.DictWriter(file, fieldnames=self.DataHeaders)
                writer.writeheader()
        else:
            print("No Connection available")

    # Function to transmit data
    def Transmit_data(self):
        TXD_data = [0]*self.TXD_datapackage_length
        TXD_data[0] = 165
        TXD_data[1] = 90
        TXD_data[2] = self.TXD_datapackage_length
        TXD_data[3] = self.Calibration_Flag
        TXD_data[4] = self.k_assist
        TXD_data_package = bytearray(TXD_data)
        # Assumil
        # ng `ser` is a serial object you would have initialized in `connect_device`
        if self.ser.is_open:
            self.ser.write(TXD_data_package)
            #print(f"Data sent: {list(data_package)}")
        else:
            print("Serial connection is not open.")

    # Function to handle command input and send data
    def stop_button_clicked(self):
        self.Slider.setValue(0)
        self.k_assist = int(self.Slider.value())
        self.Transmit_data()
        print(f"| STOP |")

    def Slider_ValueChange(self):
        self.k_assist = int(self.Slider.value())
        self.k_assist_label.setText(f"{self.k_assist*10:.0f}%")
        self.Transmit_data()
    
    def Receive_Exo_Data(self):
        #print("Receive_Exo_Data(*)")
        if self.ser.in_waiting > 0:
            print("Something received")
            if self.ser.read(1) == 165:  # 165 in uint8
                if self.ser.read(1) == 90:  # 90 in uint8
                    if self.ser.read(1) == self.RXD_datapackage_length:  # Check the length
                        print('Full Datapackage Received')
                        RXD_data = self.ser.read(self.RXD_datapackage_length-3)
                        encoder_cal_flag = RXD_data[0]
                        print(encoder_cal_flag)
                        
                        #decoded_data = [0] * (data_length // 2)  # Initialize decoded data array
                        # decode_i = 0
                        # for i in range(1, self.data_length, 2):
                        #     var = coded_data[i-1] + coded_data[i] * 256
                        #     var = (var - 65536) / 100.0 if var > 32767 else var / 100.0
                        #     self.decoded_data[decode_i] = var
                        #     decode_i += 1
                        
                        # L_leg_IMU_angle        = self.decoded_data[0]
                        # R_leg_IMU_angle        = self.decoded_data[1]
                        # L_motor_torque         = self.decoded_data[2]
                        # R_motor_torque         = self.decoded_data[3]
                        # L_motor_torque_desired = self.decoded_data[4]
                        # R_motor_torque_desired = self.decoded_data[5]
                        # t_teensy               = self.decoded_data[6]
                        # L_motor_angpos         = self.decoded_data[7]
                        # R_motor_angpos         = self.decoded_data[8]

                        # Data_Received_Flag = True
                        # if first_teensy_time:
                        #     t_0_teensy = t_teensy
                        #     first_teensy_time = False
        else:
            print('Waiting for the whole data package...')
            # ConnectButton.setText("Searching Hip Exoskeleton")
            # ConnectButton.setStyleSheet("background-color : orange")

    def Encoder_Calibration_Msg(self):
        self.msg = QMessageBox()
        self.msg.setIcon(QMessageBox.Warning)    
        self.msg.setWindowTitle("Initialization Procedure required")
        self.msg.setText("Please, follow the instructions:")
        self.msg.setInformativeText("1. Stand straight and still.\n"
                               "2. Click the Ok button.\n"
                               "3. Keep your position for 2 seconds.")
        self.msg.setDetailedText("The system must register the angular position of the thigh braces while you are standing straight in order to provide the right amount of assistance.")
        self.msg.setStandardButtons(QMessageBox.Ok)
        # Execute the message box and check if OK was clicked
        if self.msg.exec_() == QMessageBox.Ok:
            # Assign the desired value to the variable
            self.Calibration_Flag = 1  # Example variable assignment
            self.Transmit_data()
            print("Calibration confirmed:", self.Calibration_Flag)
        else:
            self.Calibration_Flag = 0  # Set to False if the message box was closed without clicking OK

    def closeEvent(self, event):
        # Call your safety function here
        self.safety_procedure()

        # Accept the close event to continue closing the window
        event.accept()

    def safety_procedure(self):
        # Code for the function to be executed before the window closes
        self.Slider.setValue(0)
        self.k_assist = int(self.Slider.value())
        self.Transmit_data()
        print("Executing safety procedure...")
        # Add any cleanup or saving actions here


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())