//To make this work: Turn on motors. Load to Teensy while subject standing still. When done (data updating in Serial Monitor), run Python code ISRA_Main.py.
//Once that one is running (data updating in Command Window), start running. To change peak intensity, change lines 85-86 in the Python code.
#include "Serial_Com.h"
#include "Wireless_IMU.h"
#include <Arduino.h>
#include "MovingAverage.h"
#include <math.h>  
#include <iomanip>  
#include <cstring>  
#include <TimeLib.h> // to get date and time
#include "Controller.h"  

/*MOTOR*/ 
#include <FlexCAN_T4.h>   
#include "Sig_Motor_Control.h"   

#define LOG_FILENAME "2024-12-17-Hip-Walking_Powered_06.csv" 

#include "ads1292r.h"    

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;   

// *** for RingBuf *** //
#include "SdFat.h"
#include "RingBuf.h" 

// Use Teensy SDIO
#define SD_CONFIG SdioConfig(FIFO_SDIO)

// Interval between points for 25 ksps.
#define LOG_INTERVAL_USEC 2000 // 500 Hz

// Size to log 10 byte lines at 25 kHz for more than ten minutes.
// #define LOG_FILE_SIZE 10*25000*600  // Size to log 10 byte lines at 25 kHz for more than ten minutes = 150,000,000 bytes.
// #define LOG_FILE_SIZE 100 * 500 * 80 // Size to log 10 byte lines at 500 Hz for more than 80 seconds =  bytes.
#define LOG_FILE_SIZE 220 * 500 * 600 // Size to log 10 byte lines at 500 Hz for more than 80 seconds = 66,000,000 bytes.

// Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
// #define RING_BUF_CAPACITY 400*512 // Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
// #define RING_BUF_CAPACITY 400 * 512 * 10 / 50 // Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
#define RING_BUF_CAPACITY 250 * 500 * 1 // Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.

// #define LOG_FILENAME "Walking.csv"
//#define LOG_FILENAME "0407_Powered_Jennifer_Walking_bd0.04_0.00_kd_1_0.6_0_0_0_0_newtau.csv"
// #define LOG_FILENAME "2022-05-24-Weibo-Walking_Powered_06.csv"

SdFs sd;
FsFile file;  

// RingBuf for File type FsFile.
RingBuf<FsFile, RING_BUF_CAPACITY> rb;    

ads1292r torque_sensor1;     


// Settings
int assist_mode = 1;
int stopFlag = 0;
int saveDataFlag = 1;
int numOfInitialSteps = 1;
int enableExtensionStop = 0;
uint32_t Motor_ID1 = 1; // Motor Can Bus ID, left leg, positive current = extension = encoder value increase // updated on 2023-05-16 2
uint32_t Motor_ID2 = 2; // Motor Can Bus ID, right leg, positive current = flexion = encoder value decrease // updated on 2023-05-16 3
int CAN_ID = 3;         // CAN port from Teensy
double Gear_ratio = 9;  //The actuator gear ratio, will enfluence actuator angle and angular velocity
double torque_constant_before_gear = 0.105; // before gear at 24 V. Ref: https://store.tmotor.com/goods.php?id=982
double torqueCorrectionFactor = 1.09; // actual torque is 1.09x the motor-returned torque through experiments (2023/07/14)
int triggerPin = A9; //reading pin for sync with external mocap device
int groundPin = A8; // for sync with external mocap device                                 //Create torque sensor object see ads1292r.h


// Data logging
int isLogging = 0;   
int L_load_cell_torque = 0;  
int R_load_cell_torque = 0;    

/*Filter*/
MovingAverage LTAVx(12);    
MovingAverage RTAVx(12);     
float f_LTAVx = 0;  
float f_RTAVx = 0;  

CAN_message_t msgR;   

int Sig_Motor_ID_1 = 1;     
int Sig_Motor_ID_2 = 0;       

double torque_command = 0;  
double velocity_command = 0;  
double position_command = 0;  

double M1_torque_command = 0;  
double M2_torque_command = 0;   

double RL_torque_command_1 = 0.0;    
double RL_torque_command_2 = 0.0;   

double MAX_torque_command = 8;   
double MIN_torque_command = -8;    

int LimitInf = -18;    
int LimitSup = 18;    

float p_des = 0;   
float v_des = 0;    
float kp = 0;   
float kd = 0;   
float t_ff = 0;   

/*MOTOR*/    
float initial_pos_1 = 0;       
float initial_pos_2 = 0;       

Motor_Control_Tmotor sig_m1(0x000, CAN_ID);   
Motor_Control_Tmotor sig_m2(0x001, CAN_ID);   
/*MOTOR*/  

Controller controller;  

/*Isra Serial Class Setup*/  
Serial_Com Serial_Com;   

/*Sensors Setup*/ 
IMU imu;     

////// sensor  
// ads1292r torque_sensor1;    //FOR THE TORQUE SENSORS

/*Serial Send*/  
size_t Send_Length = 11; 
char Send[11] = { 0x31, 0x32, 0x32, 0x33, 0x33,
                  0x30, 0x31, 0x32, 0x33, 0x33,
                  0x33 };   

/*iMU SEND*/
uint16_t L_IMUX_int = 0x00;  
uint16_t R_IMUX_int = 0x00;   

uint16_t L_IMUV_int = 0x00;
uint16_t R_IMUV_int = 0x00; 

uint16_t L_CMD_int16 = 0x7fff;  
float L_CMD_serial   = 0.0;  

uint16_t R_CMD_int16 = 0x7fff;
float R_CMD_serial   = 0.0;

float IMUX_float = 0;   
float IMU11 = 0;   
float IMU22 = 0;   
float IMU33 = 0;  
float IMU44 = 0;  

/* Time control*/
// unsigned long Delta_T1 = 35;  //Looks like increasing this improves stability, but mkaes the torque less smooth
// unsigned long t_i, t_pr1;
// unsigned long beginning = 0;
double t;   
double next_t;    
double delta_t;    

//***For managing the Controller and Bluetooth rate
unsigned long t_0 = 0;
// double cyclespersec_ctrl = 28;  // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus) Controller must be faster than ble
double cyclespersec_ctrl = 100;    // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus) Controller must be faster than ble
double cyclespersec_ble  = 20;     // [Hz] Bluetooth sending data frequency 
unsigned long current_time = 0;
unsigned long previous_time = 0;                                           // used to control the controller sample rate.
unsigned long previous_time_ble = 0;                                       // used to control the Bluetooth communication frequency
unsigned long Tinterval_ctrl_micros = (unsigned long)(1000000 / cyclespersec_ctrl); // used to control the teensy controller frequency
unsigned long Tinterval_ble_micros  = (unsigned long)(1000000 / cyclespersec_ble);  // used to control the Bluetooth communication frequency
//**********************************

double Fsample = 500;        // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus)
double Fsample_ble = 100;    // [Hz] Bluetooth sending data frequency                                        // used to control the Bluetooth communication frequency
unsigned long Tinterval_microsecond = (unsigned long)(1000000 / Fsample);               // used to control the teensy controller frequency
unsigned long Tinterval_ble_microsecond = (unsigned long)(1000000 / Fsample_ble);       // used to control the Bluetooth communication frequency


//***Data sent via bluetooth
char datalength_ble = 32;      // Bluetooth Data Length (32)
char data_ble[60] = {0};       // Data array for bluetooth data sending:  Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)->bluetooth->Adafruit Feather nRF52840 Express(central)->usb->computer
char data_rs232_rx[60] = {0};  // Data array for bluetooth data receive:  computer->USB->Adafruit Feather nRF52840 Express(central)->bluetooth->Adafruit Feather nRF52840 Express(peripheral)->RS232->Teensy

int L_leg_IMU_angle = 0;       
int R_leg_IMU_angle = 0;  
int L_motor_torque  = 0;   
int R_motor_torque  = 0;  
int L_motor_torque_desired = 0;    
int R_motor_torque_desired = 0;   
int t_teensy = 0;  
int M_Selected = 0;  
int CtrlMode_Selected = 0;  

int GUI_force_cmd     = 0;  
int GUI_stiffness_cmd = 0;  
int GUI_damping_cmd   = 0;  
int GUI_assistive_ratio_cmd = 0;     

int GUI_pos_ampl_cmd    = 0;       
int GUI_pos_fre_cmd     = 0;       
int GUI_force_ampl_cmd  = 10;        
int GUI_force_fre_cmd   = 10;       

double GUI_force = 1.0;    
double GUI_K_p   = 1.0;    
double GUI_K_d   = 0.1;    

double assistive_ratio = 0.08;   

int L_pos_int_d = 0;     
int L_pos_int_a = 0;     
int L_vel_int_d = 0;       
int L_vel_int_a = 0;      
 
int R_pos_int_d = 0;     
int R_pos_int_a = 0;       
int R_vel_int_d = 0;         
int R_vel_int_a = 0;       
//**************************

double cmd_ampl = 1.0;        
double cmd_fre  = 1.0;      
float pos_ampl  = 0.0;      
float pos_fre   = 0.5;      

float l_pos_des = 0.0;    
float l_vel_des = 0.0;    
float r_pos_des = 0.0;     
float r_vel_des = 0.0;     

float ref_force_ampl = 0.2;    
float ref_force_fre = 0.5;    

float l_ref_tau    = 0.0;   
float l_ref_tau_dt = 0.0;    
float r_ref_tau    = 0.0;     
float r_ref_tau_dt = 0.0;    

float l_leg_angle    = 0.0;  
float r_leg_angle    = 0.0;  
float l_leg_velocity = 0.0;   
float r_leg_velocity = 0.0;  

//***Impedance Control Test***//  
float tau_imp = 0.0;      
float kp_imp = 1.0;     
float kd_imp = 0.01 * kp_imp;         
//***Impedance Control Test***//    

//***Torque Control Test */
float dt = 0.01;    

float tau_t_1 = 0.0;  
float tau_t_1_last = 0.0;    
float tau_t_2 = 0.0;    
float tau_t_2_last = 0.0;   

float tau_dt_1 = 0.0;    
float tau_dt_2 = 0.0;     

float tau_ff_1 = 0.0;      
float tau_ff_2 = 0.0;   

//*** Motor Mode Set ***//   
int ctl_method = 1;    // 0 for using RL controller, 1 for using other normal controller  
int ctl_mode = 0;      // 0 for torque control, 1 for mit control    
int ctl_type = 0;      // 0 for motion, 1 for force tracking, 2 for direct torque   

int sensor_type = 0;   // 0 for using IMU, 1 for using encoder   
int l_ctl_dir = 1;      
int r_ctl_dir = -1;    
float torque_cmd_scale = 20.0;  
//*** Motor Mode Set ***//    

int doi          = 0;   
int currentpoint = 0;    
int delayindex   = 0;    
// double Rescaling_gain    = 0.01; // 1.6 for max 4 Nm
double LTx_filtered      = 0;
double LTx_filtered_last = 0;
double RTx_filtered      = 0;
double RTx_filtered_last = 0;
double torque_filtered   = 0;
double RLTx              = 0;
double RLTx_filtered     = 0;
double RLTx_delay[100]   = {};
double torque_delay[100] = {};  

int L_motor_torque_command = 0;
int R_motor_torque_command = 0;
double Rescaling_gain    = 0;
double Flex_Assist_gain  = 0;
double Ext_Assist_gain   = 0;
double Assist_delay_gain = 0;  

double S_torque_command_left = 0.0;    
double S_torque_command_right = 0.0;   

double Cur_command_L = 0;
double Cur_command_R = 0;  

int LK_ble = 0;                //left knee angle
int RK_ble = 0;                //right knee angle
int current_command_L_ble = 0; //current reference(A) for inner loop current control
int current_command_R_ble = 0;
int current_actual_L_ble = 0;
int current_actual_R_ble = 0;
int torque_command_L_ble = 0; //total torque reference(N-m)
int torque_command_R_ble = 0;
int torque_command_imp_L_ble = 0; // impedance torque
int torque_command_imp_R_ble = 0; // impedance torque
int torque_command_ff_L_ble = 0;  // feedforward torque
int torque_command_ff_R_ble = 0;  // feedforward torque
int torque_estimated_L_ble = 0; //total torque reference(N-m)
int torque_estimated_R_ble = 0;
int torque_measured_L_ble = 0;             //actual torque(N-m)(measured by torque sensor)
int torque_measured_R_ble = 0;
int motor_speed_L_ble = 0;
int motor_speed_R_ble = 0;
double torque_command_L = 0;
double torque_command_R = 0;

double torque_measured_L = 0.0;
double torque_measured_R = 0.0;  

int gait_percentage_L_ble = 0;
int insole_torque_command_L_ble = 0;
int insole_torque_command_R_ble = 0;
int insole_gait_percent_L_ble = 0;
int insole_gait_percent_R_ble = 0;
int imu_gait_percent_L_ble = 0;
int imu_gait_percent_R_ble = 0;
int imu_gait_percent_imp_L_ble = 0; // impedance torque
int imu_gait_percent_imp_R_ble = 0; // mpedance torque
int imu_gait_percent_ff_L_ble = 0;  // eedforward biological torque
int imu_gait_percent_ff_R_ble = 0;  // feedforward biological torque
double Insole_gain = 5;

int Stop_button = 0;    // Stop function
String mode = "start";
double milli_time_current_double = 0;  

double relTime = 0.0;   
int SDcounter = 0;  
int SDtotalCounter = 0;   

// Send message to MATLAB GUI
int messageID = 0;
int isSDCardPresent = 0;
int isDataSaved = 0;
int logYear;
int logMonth;
int logDay;
int logHour;
int logMinute;
int logSecond;

int current_limitation = 10; //(unit Amp)//double Tor_command_L = 0;
// Trigger
int triggerOn = 0;
int triggerVal = 0;  // analog trigger pin value
// Data logging
int taskIdx = 1;
int conditionIdx = 1; // baseline=1, sham=2, powered=3
int trialIdx = 1;
String taskName;
String conditionName;
String logFileName;
String paramFileName;


//// setup can and motors ////
void setup() {
  delay(3000);   

  Serial.begin(115200);     //115200/9600=12
  Serial2.begin(115200);    //115200/9600=12
  //Serial7.begin(115200);  // Communication with Raspberry PI or PC for High-lever controllers like RL
  Serial5.begin(115200);    //used to communication with bluetooth peripheral. Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)
  
  Serial_Com.INIT();    

  //#################
  // Serial.println("SETUP DONE");  
  // Serial.print("Controller executed at ");   
  // Serial.print(String(cyclespersec_ctrl));   
  // Serial.println(" Hz");   
  // Serial.print("BT com executed at ");   
  // Serial.print(String(cyclespersec_ble));   
  // Serial.println(" Hz");   
  //#################### 

  torque_sensor1.Torque_sensor_initial();                                           //initial the torque sensor see ads1292r.cpp.
  torque_sensor1.Torque_sensor_gain(0.0003446 * (-1) * 2, 0.0003446 * (-1) * 2.35); //set the calibration gain for torque sensor. Torque= gain* ADCvalue+offset.see ads1292r.cpp.
  torque_sensor1.Torque_sensor_offset_calibration();              

  delay(1000);  

  SPI1.setMOSI(26);  
  SPI1.setMISO(1);  
  SPI1.setSCK(27);   

  initial_CAN();    
  initial_Sig_motor();    
  delay(100);  
  IMUSetup();   

  t_0 = micros();    
}  

//// initial sig motor //// 
void initial_Sig_motor() {  
  // sig_m1.error_clear();    
  // delay(200); 

  // sig_m1.reboot();  
  // delay(200);  

  // sig_m1.sig_motor_reset();   
  // sig_m2.sig_motor_reset();   
  // delay(1000);  

  // // delay(1000);  
  // sig_m1.sig_encoder_reset();    
  // sig_m2.sig_encoder_reset();    
  // delay(10000);   

  /////////// set control mode /////////  
  if (ctl_mode == 1)  
  {
    sig_m1.sig_mit_ctl_mode_start();      
    sig_m2.sig_mit_ctl_mode_start();     
  } 
  else
  {
    sig_m1.sig_torque_ctl_mode_start();    
    delay(200);   
    sig_m2.sig_torque_ctl_mode_start();        
  } 
  delay(200);  
  
  sig_m1.sig_motor_start();    
  sig_m1.request_pos_vel();    
  delay(500);   

  sig_m2.sig_motor_start();    
  sig_m2.request_pos_vel();     
  delay(500);    

  if (ctl_mode == 1)  
  {
    sig_m1.sig_mit_ctl_cmd(0.0, 0.0, 0.0, 0.0, 0.01);     
    sig_m2.sig_mit_ctl_cmd(0.0, 0.0, 0.0, 0.0, 0.01);          
    receive_mit_ctl_feedback();     
  }
  else{
    // sig_m1.request_torque();   
    sig_m1.sig_torque_cmd(0.01);    
    delay(200);    
    // sig_m2.request_torque();   
    sig_m2.sig_torque_cmd(0.01);      
    delay(200);   
  } 

  for (int i =0; i < 1000; i++)
  {
    receive_torque_ctl_feedback();     
  }
  delay(1000);   

  initial_pos_1 = sig_m1.pos;     
  initial_pos_2 = sig_m2.pos;     

  delay(500);  

  /////// command initial setting ///////
  M1_torque_command = 0.0;         
  M2_torque_command = 0.0;         
}

// Zhimin Hou for Sig Motor ////   
void loop() {  
  Gait_based_torque();  
}  

// 
void Samsung_torque()  
{
    RLTx = imu.RTx - imu.LTx; // right-left thigh angle difference
    
    LTx_filtered_last = LTx_filtered;
    LTx_filtered      = ((0.95 * LTx_filtered_last) + (0.05 * imu.LTx));
    
    RTx_filtered_last = RTx_filtered;  
    RTx_filtered      = ((0.95 * RTx_filtered_last) + (0.05 * imu.RTx));   

    RLTx_filtered     = RTx_filtered - LTx_filtered; 
    RLTx_delay[doi]   = RLTx_filtered;  
    torque_filtered   = (sin(RTx_filtered * PI / 180) - sin(LTx_filtered * PI / 180));
    torque_delay[doi] = torque_filtered;  

    currentpoint = doi;  
    delayindex   = doi - Assist_delay_gain;  

    if (delayindex < 0) {
      delayindex = delayindex + 100;  
    }
    else if (delayindex >= 100) {
      delayindex = delayindex - 100;  
    }

    doi++;
    doi = doi % 100;
    
    if ((RLTx_delay[delayindex] >= 0) && (RLTx_delay[delayindex] < 120)) {
      M1_torque_command = - Rescaling_gain * Ext_Assist_gain * torque_delay[delayindex]; // left hip torque
      M2_torque_command = Rescaling_gain * Flex_Assist_gain * torque_delay[delayindex];  // right hip torque
    }
    else if ((RLTx_delay[delayindex] < 0) && (RLTx_delay[delayindex] > -120)) {
      M2_torque_command = Rescaling_gain * Ext_Assist_gain * torque_delay[delayindex];  // right hip torque
      M1_torque_command = -Rescaling_gain * Flex_Assist_gain * torque_delay[delayindex]; // left hip torque
    }
    else {
      // M2_torque_command=0;
      // M1_torque_command=0;
    }

    // int max_allowed_torque = 30;   
}

void Gait_based_torque()
{
  imu.READ();                          //Check if IMU data available and read it. the sample rate is 100 hz
  current_time = micros();             //query current time (microsencond)
  //********* use to control the teensy controller frequency **********//
  
  if (current_time - previous_time > Tinterval_microsecond) // check if the time period of control loop is already larger than Sample period of control loop (Tinterval_microsecond)
  {
    imu.READ();
    if (Stop_button) //stop
    {
      p_des = 0; //dont change this
      v_des = 0; //dont change this
      kp = 0; //dont change this
      kd = 0;   //dont change this
      torque_command_L = 0; // torque command
      torque_command_R = 0; // torque command
    }
    else
    {
      Compute_Torque_Commands(); // TODO
    }

    // if (assist_mode == 10)
    // {
    //   // enforceInitialSteps();
    // }
    for (int i =0; i < 4; i++)   
    {
      receive_torque_ctl_feedback();     
    }

    // if (isLogging)
    // {
    //   logData3();  
    // }  

    M2_torque_command = torque_command_R * r_ctl_dir;        /// for right.   
    M1_torque_command = torque_command_L * l_ctl_dir;       /// left        

    sig_m1.sig_torque_cmd(M1_torque_command);     
    sig_m2.sig_torque_cmd(M2_torque_command);        

    previous_time = current_time; //reset previous control loop time
    relTime += Tinterval_microsecond / 1000;
  }

  //********* use to control the Bluetooth communication frequency **********//
  if (current_time - previous_time_ble > Tinterval_ble_microsecond)
  {
    receive_ble_Data();
    send_ble_Data(); // send the BLE data
    previous_time_ble = current_time;
     
    // CustomWait(1000);
    // torque_sensor1.Torque_sensor_read(); //FOR THE TORQUE SENSORS
    // torque_measured_L = -torque_sensor1.torque[0];
    // torque_measured_R = torque_sensor1.torque[1];
    // // Serial.print(-torque_sensor1.torque[0]); Serial.print("   ");//FOR THE TORQUE SENSORS. THIS IS THE LEFT MOTOR
    // // Serial.println(torque_command_L); 
    // // plot_controller_data();  
  }
  if (Serial.available())
  {
    char cmd = (char)Serial.read();
    if (cmd == 's')
    {
      stopFlag = 1;
      Serial.println("Stopped");
      if (saveDataFlag)
      {
        SDCardSaveToFile();
        messageID = 2;
        int messageValueArray[] = {1, logYear, logMonth, logDay, logHour, logMinute, logSecond};
        sendMessage(messageID, messageValueArray, 7);
        String yearStr = String(logYear);
        String monthStr = (logMonth < 10) ? "0" + String(logMonth) : String(logMonth);
        String dayStr = (logDay < 10) ? "0" + String(logDay) : String(logDay);
        String hourStr = (logHour < 10) ? "0" + String(logHour) : String(logHour);
        String minuteStr = (logMinute < 10) ? "0" + String(logMinute) : String(logMinute);
        String secondStr = (logSecond < 10) ? "0" + String(logSecond) : String(logSecond);
        String timeStr = yearStr + "-" + monthStr + "-" + dayStr + "-" + hourStr + "-" + minuteStr + "-" + secondStr;

        String stringOne = taskName + '-';
        String stringTwo = stringOne + conditionName + "-Trial";
        logFileName = timeStr + "-" + stringTwo + String(trialIdx) + ".csv";
        paramFileName = timeStr + "-" + stringTwo + String(trialIdx) + "-Parameters.txt";
        Serial.println(logFileName);
        // SaveAssistanceProfileParameters(paramFileName.c_str());
      }
    }
  }
}

void receive_ble_Data()
{
  if (Serial5.available() >= 20)
  {
    // Serial.println("-------------New data received-------------------");
    data_rs232_rx[0] = Serial5.read();
    if (data_rs232_rx[0] == 165)
    {
      data_rs232_rx[1] = Serial5.read();
      if (data_rs232_rx[1] == 90)
      {
        data_rs232_rx[2] = Serial5.read();
        if (data_rs232_rx[2] == 20)
        {
          Serial5.readBytes(&data_rs232_rx[3], 17);
          if (data_rs232_rx[3] == 0)
          {
            Stop_button = int(data_rs232_rx[4]);
            if (Stop_button)
            {
              Serial.println("STOP button pressed");
            }
            else
            {
              Serial.println("START button pressed");
            }
          }
          else if (data_rs232_rx[3] == 1)
          {
            assist_mode = int(data_rs232_rx[4]);
            Serial.print("Mode: ");
            Serial.println(assist_mode);
            //Serial.print("    ");
            //Serial.println(mode);
          }
          else if (data_rs232_rx[3] == 2)
          {
            float Gain_E = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
            imu.Gain_E = Gain_E;

            Serial.print("Extension gain from Matlab: ");
            Serial.println(Gain_E);
          }
          else if (data_rs232_rx[3] == 3)
          {
            float Gain_F = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
            imu.Gain_F = Gain_F;

            Serial.print("Flexion gain from matlab: ");
            Serial.println(Gain_F);
          }
          else if (data_rs232_rx[3] == 4)
          { // 10 ms per timepoint, delaypoint needs to be less than 100
            // 1 delaypoint in matlab interface equals 5 timepoints here, that is 50 ms
            int delaypoint = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000;
            imu.delaypoint = delaypoint;

            Serial.print("Delay [ms]: ");
            Serial.println(delaypoint * 10);
            // delay = delaypoint*sample time
          }
          // else if (data_rs232_rx[3] == 5)
          // {
          //   weight = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
          //   Serial.print("Weight [kg]: ");
          //   Serial.println(weight);
          // }
          // else if (data_rs232_rx[3] == 6)
          // {
          //   Insole_gain = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
          //   Serial.print("Insole gain: ");
          //   Serial.println(Insole_gain);
          // }
          else if (data_rs232_rx[3] == 7)
          {
            // reset_motor_angle();
            // int m = 0;
            // float sum = 0;
            // int SSize = 20;
            // while (m < SSize) {
            //   sum = (m1.pos + m2.pos) + sum;
            //   m++;
            // }
            // neturalKneeAngle = 0; //sum / (SSize * 2); 
            Serial.println("The angle of motor has been reset");
            imu.INIT_MEAN();
            Serial.println("The angle of IMUs has been reset");
          }

          // else if (data_rs232_rx[3] == 11)
          // {
          //   float Gain_Sw = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
          //   controller.ControllerGain_L = Gain_Sw;
          //   controller.ControllerGain_R = Gain_Sw;
          //   Serial.print("Swing gain from matlab: ");
          //   Serial.println(Gain_Sw);
          // }
          // else if (data_rs232_rx[3] == 12)
          // {
          //   float Gain_St = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
          //   controller.FeedForwardGain_L = Gain_St;
          //   controller.FeedForwardGain_R = Gain_St;
          //   Serial.print("Stance gain from matlab: ");
          //   Serial.println(Gain_St);
          // }
          // else if (data_rs232_rx[3] == 13)
          // {
          //   float Timing_Sw = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
          //   controller.P_ahead_imp = Timing_Sw;
          //   Serial.print("Swing timing from matlab: ");
          //   Serial.println(Timing_Sw);
          // }
          // else if (data_rs232_rx[3] == 14)
          // {
          //   float Timing_St = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
          //   controller.P_ahead_ff = Timing_St;
          //   Serial.print("Stance timing from matlab: ");
          //   Serial.println(Timing_St);
          // }
          // else if (data_rs232_rx[3] == 15)
          // {
          //   double STSMagnitude = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
          //   imu.STS_Gain = STSMagnitude;
          //   Serial.print("Sit-to-stand gain from matlab: ");
          //   Serial.println(STSMagnitude);
          // }
          // else if (data_rs232_rx[3] == 16)
          // {
          //   double STSSensitivity = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
          //   imu.STSSlopeThreshold = STSSensitivity;
          //   Serial.print("Sit-to-stand slope threshold from matlab: ");
          //   Serial.println(STSSensitivity);
          // }
          // else if (data_rs232_rx[3] == 17)
          // {
          //   double STSAlpha = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
          //   imu.alpha = STSAlpha;
          //   Serial.print("Sit-to-stand alpha from matlab: ");
          //   Serial.println(STSAlpha);
          // }
          // else if (data_rs232_rx[3] == 18)
          // {
          //   double STSBeta = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
          //   imu.beta = STSBeta;
          //   Serial.print("Sit-to-stand beta from matlab: ");
          //   Serial.println(STSBeta);
          // }
          else if (data_rs232_rx[3] == 20)
          {
            isLogging = int(data_rs232_rx[7]);
            if (isLogging == 1)
            {
              taskIdx = int(data_rs232_rx[4]);
              conditionIdx = int(data_rs232_rx[5]);
              trialIdx = int(data_rs232_rx[6]);
              // String taskName;
              if (taskIdx == 1)
              {
                taskName = "Walking";
              }
              else if (taskIdx == 2)
              {
                taskName = "STS";
              }
              String stringOne = taskName + '-';

              // String conditionName;
              if (conditionIdx == 1)
              {
                conditionName = "Baseline";
              }
              else if (conditionIdx == 2)
              {
                conditionName = "Sham";
              }
              else if (conditionIdx == 3)
              {
                conditionName = "Powered";
              }
              else
              {
                conditionName = "Others";
              }
              logYear = year();
              logMonth = month();
              logDay = day();
              logHour = hour();
              logMinute = minute();
              logSecond = second();
              String yearStr = String(logYear);
              String monthStr = (logMonth < 10) ? "0" + String(logMonth) : String(logMonth);
              String dayStr = (logDay < 10) ? "0" + String(logDay) : String(logDay);
              String hourStr = (logHour < 10) ? "0" + String(logHour) : String(logHour);
              String minuteStr = (logMinute < 10) ? "0" + String(logMinute) : String(logMinute);
              String secondStr = (logSecond < 10) ? "0" + String(logSecond) : String(logSecond);
              String timeStr = yearStr + "-" + monthStr + "-" + dayStr + "-" + hourStr + "-" + minuteStr + "-" + secondStr;

              String stringTwo = stringOne + conditionName + "-Trial";
              logFileName = timeStr + "-" + stringTwo + String(trialIdx) + ".csv";
              paramFileName = timeStr + "-" + stringTwo + String(trialIdx) + "-Parameters.txt";
              // Serial.println(logFileName);
              // int status = SDCardSetup(logFileName.c_str());
              // if (status == 1)
              // {
              //   messageID = 1;
              //   int messageValueArray[] = {1};
              //   sendMessage(messageID, messageValueArray, 1);
              //   relTime = 0.0;
              //   Serial.println("Data logging started......");
              // }
              // else
              // {
              //   messageID = 1;
              //   int messageValueArray[] = {0};
              //   sendMessage(messageID, messageValueArray, 1);
              //   isLogging = 0; // prevent logging because there is no SD
              //   Serial.println("Data logging aborted because there is no SD");
              // }

            }
            else if (isLogging == 0)
            {
              SDCardSaveToFile();
              messageID = 2;
              int messageValueArray[] = {1, logYear - 2000, logMonth, logDay, logHour, logMinute, logSecond};
              sendMessage(messageID, messageValueArray, 7);
              Serial.println("Data logging stopped......");
              // SaveAssistanceProfileParameters(paramFileName.c_str());
            }
          }
          // else if (data_rs232_rx[3] == 25)
          // {
          //   int sineAssistanceTypeIdx = int(data_rs232_rx[4]);
          //   String sineAssistanceTypeName;
          //   if (sineAssistanceTypeIdx == 1)
          //   {
          //     sineAssistanceTypeName = "Flexion";
          //   }
          //   else if (sineAssistanceTypeIdx == 2)
          //   {
          //     sineAssistanceTypeName = "Extension";
          //   }
          //   double sineAssistanceMagnitude = ((int16_t)(((uint16_t)data_rs232_rx[5]) | ((uint16_t)data_rs232_rx[6] << 8))) / 100.0;
          //   int sineAssistanceShift = int(data_rs232_rx[7]);
          //   int sineAssistanceDuration = int(data_rs232_rx[8]);
          //   double sineAssistanceSaturation = ((int16_t)(((uint16_t)data_rs232_rx[9]) | ((uint16_t)data_rs232_rx[10] << 8))) / 100.0;
          //   controller.addSineAssistanceProfile(sineAssistanceTypeIdx, sineAssistanceMagnitude, sineAssistanceShift, sineAssistanceDuration, sineAssistanceSaturation);
          //   Serial.print("Added ");
          //   Serial.print(sineAssistanceTypeName);
          //   Serial.print(" assistance. Magnitude = ");
          //   Serial.print(sineAssistanceMagnitude);
          //   Serial.print(", shift = ");
          //   Serial.print(sineAssistanceShift);
          //   Serial.print(", duration = ");
          //   Serial.print(sineAssistanceDuration);
          //   Serial.print(", saturation = ");
          //   Serial.print(sineAssistanceSaturation);
          //   Serial.print(". Number of flexion profile = ");
          //   Serial.print(controller.sineAssistanceFlexionProfileCounter);
          //   Serial.print(". Number of extension profile = ");
          //   Serial.println(controller.sineAssistanceExtensionProfileCounter);
          // }
          // else if (data_rs232_rx[3] == 26)
          // {
          //   int sineAssistanceTypeIdx = int(data_rs232_rx[4]);
          //   String sineAssistanceTypeName;
          //   if (sineAssistanceTypeIdx == 1)
          //   {
          //     sineAssistanceTypeName = "Flexion";
          //   }
          //   else if (sineAssistanceTypeIdx == 2)
          //   {
          //     sineAssistanceTypeName = "Extension";
          //   }
          //   int sineAssistanceProfileIdx = int(data_rs232_rx[5]);
          //   // for (int i = 0; i < 3; i++)
          //   // {
          //   //   Serial.print(controller.sineAssistanceExtensionParameterList[0][i]);
          //   //   Serial.print(" ");
          //   // }
          //   // Serial.println(" ");
          //   controller.removeSineAssistanceProfile(sineAssistanceTypeIdx, sineAssistanceProfileIdx);
          //   Serial.print("Removed ");
          //   Serial.print(sineAssistanceTypeName);
          //   Serial.print(" assistance. ID = ");
          //   Serial.println(sineAssistanceProfileIdx);

          //   // for (int i = 0; i < 3; i++)
          //   // {
          //   //   Serial.print(controller.sineAssistanceExtensionParameterList[0][i]);
          //   //   Serial.print(" ");
          //   // }
          //   // Serial.println(" ");
          // }
          // else if (data_rs232_rx[3] == 27)
          // {
          //   int sineAssistanceTypeIdx = int(data_rs232_rx[4]);
          //   String sineAssistanceTypeName;
          //   if (sineAssistanceTypeIdx == 1)
          //   {
          //     sineAssistanceTypeName = "Flexion";
          //   }
          //   else if (sineAssistanceTypeIdx == 2)
          //   {
          //     sineAssistanceTypeName = "Extension";
          //   }
          //   int sineAssistanceProfileIdx = int(data_rs232_rx[5]);
          //   double sineAssistanceMagnitude = ((int16_t)(((uint16_t)data_rs232_rx[6]) | ((uint16_t)data_rs232_rx[7] << 8))) / 100.0;
          //   int sineAssistanceShift = int(data_rs232_rx[8]);
          //   int sineAssistanceDuration = int(data_rs232_rx[9]);
          //   double sineAssistanceSaturation = ((int16_t)(((uint16_t)data_rs232_rx[10]) | ((uint16_t)data_rs232_rx[11] << 8))) / 100.0;
          //   controller.editSineAssistanceProfile(sineAssistanceTypeIdx, sineAssistanceProfileIdx, sineAssistanceMagnitude, sineAssistanceShift, sineAssistanceDuration, sineAssistanceSaturation);
          //   Serial.print("Edit ");
          //   Serial.print(sineAssistanceTypeName);
          //   Serial.print(" assistance. ID = ");
          //   Serial.print(sineAssistanceProfileIdx);
          //   Serial.print(". Magnitude = ");
          //   Serial.print(sineAssistanceMagnitude);
          //   Serial.print(", shift = ");
          //   Serial.print(sineAssistanceShift);
          //   Serial.print(", duration = ");
          //   Serial.print(sineAssistanceDuration);
          //   Serial.print(", saturation = ");
          //   Serial.println(sineAssistanceSaturation);
          // }
          // else if (data_rs232_rx[3] == 28)
          // {
          //   stopAssistanceInSwingAtNeturalKnee = int(data_rs232_rx[4]); //set to be always on
          //   Serial.println(stopAssistanceInSwingAtNeturalKnee);
          // }
          // else if (data_rs232_rx[3] == 30)
          // {
          //   double STSStandUpMagnitude = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 100.0;
          //   imu.STSStandUpMagnitude = STSStandUpMagnitude;
          //   Serial.print("STS magnitude (stand up) = ");
          //   Serial.print(STSStandUpMagnitude);
          //   Serial.println(" Nm");
          // }
          // else if (data_rs232_rx[3] == 31)
          // {
          //   double STSSitDownMagnitude = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 100.0;
          //   imu.STSSitDownMagnitude = STSSitDownMagnitude;
          //   Serial.print("STS magnitude (sit down) = ");
          //   Serial.print(STSSitDownMagnitude);
          //   Serial.println(" Nm");
          // }
          // else if (data_rs232_rx[3] == 32)
          // {
          //   double STSStandSlackAngle = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 100.0;
          //   imu.STSStandSlackAngle = STSStandSlackAngle;
          //   Serial.print("STS slack angle (stand) = ");
          //   Serial.print(STSStandSlackAngle);
          //   Serial.println(" deg");
          // }
          // else if (data_rs232_rx[3] == 33)
          // {
          //   double STSSitSlackAngle = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 100.0;
          //   imu.STSSitSlackAngle = STSSitSlackAngle;
          //   Serial.print("STS slack angle (sit) = ");
          //   Serial.print(STSSitSlackAngle);
          //   Serial.println(" deg");
          // }
          // else if (data_rs232_rx[3] == 34)
          // {
          //   double STSStandThreshold = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 100.0;
          //   imu.STSStandThreshold = STSStandThreshold;
          //   Serial.print("STS threshold (stand) = ");
          //   Serial.print(STSStandThreshold);
          //   Serial.println(" deg");
          // }
          // else if (data_rs232_rx[3] == 35)
          // {
          //   double STSSitThreshold = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 100.0;
          //   imu.STSSitThreshold = STSSitThreshold;
          //   Serial.print("STS slack angle (sit) = ");
          //   Serial.print(STSSitThreshold);
          //   Serial.println(" deg");
          // }
          // else if (data_rs232_rx[3] == 36)
          // {
          //   imu.STSStandUpMagnitude = 0.0;
          //   imu.STSSitDownMagnitude = 0.0;
          //   imu.STSStandSlackAngle = 7.5;
          //   imu.STSSitSlackAngle = 7.5;
          //   imu.STSStandThreshold = 5.0;
          //   imu.STSSitThreshold = 75.0;
          //   imu.STS_state = 0;
          //   imu.STSSitPreviousMaxAngle = 75;
          //   imu.STSStandPreviousMinAngle = 5;
          //   Serial.println("STS Reset button pressed. All parameters reset to default values.");
          // }
          else if (data_rs232_rx[3] == 50) // Sine controller
          {
            double sineAssistanceMagnitude = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 100.0;
            int sineAssistanceShift = int(data_rs232_rx[6]);
            int sineAssistanceDuration = int(data_rs232_rx[7]);

            controller.updateSineAssistanceProfile(sineAssistanceMagnitude, sineAssistanceShift, sineAssistanceDuration);
            Serial.print("Sine assistance. Magnitude = ");
            Serial.print(sineAssistanceMagnitude);
            Serial.print(", shift = ");
            Serial.print(sineAssistanceShift);
            Serial.print(", duration = ");
            Serial.println(sineAssistanceDuration);
           
          }
        }
      }
    }
  }
}

void send_ble_Data()
{
  // Serial.println("Sending...");
  LK_ble = imu.LKx * 100;
  RK_ble = imu.RKx * 100;

  current_command_L_ble = -Cur_command_L * 100; //
  current_command_R_ble = Cur_command_R * 100;  //
  // current_command_L_ble = -m1.iq_A * 100;
  // current_command_R_ble = m2.iq_A * 100;

  torque_command_L_ble = torque_command_L * 100; //
  torque_estimated_L_ble = sig_m1.torque * 100;  //
  torque_measured_L_ble = torque_measured_L * 100;

  torque_command_R_ble = -torque_command_R * 100; //
  torque_estimated_R_ble = sig_m2.torque * 100;
  torque_measured_R_ble = torque_measured_R * 100;

  //  gait_percentage_L_ble = imu.gait_percentage_L * 100;
  gait_percentage_L_ble = 0;

  // insole_torque_command_L_ble = insole.normalized_torque_command_L * 100;
  // insole_torque_command_R_ble = insole.normalized_torque_command_R * 100;
  // insole_gait_percent_L_ble = insole.gait_percent_L * 100;
  // insole_gait_percent_R_ble = insole.gait_percent_R * 100;
  imu_gait_percent_L_ble = controller.GP_IMU_L_ahead * 100;
  imu_gait_percent_R_ble = controller.GP_IMU_R_ahead * 100;
  //
  imu_gait_percent_imp_L_ble = controller.GP_IMU_L_ahead_imp * 100;
  imu_gait_percent_imp_R_ble = controller.GP_IMU_R_ahead_imp * 100;
  imu_gait_percent_ff_L_ble = controller.GP_IMU_L_ahead_ff * 100;
  imu_gait_percent_ff_R_ble = controller.GP_IMU_R_ahead_ff * 100;
  //
  torque_command_imp_L_ble = -controller.impedanceTorque_L * 100;
  torque_command_imp_R_ble = -controller.impedanceTorque_R * 100;
  torque_command_ff_L_ble = controller.feedforwardTorque_L * 100;
  torque_command_ff_R_ble = controller.feedforwardTorque_R * 100;
  //
  motor_speed_L_ble = sig_m1.spe * 100; // radian
  motor_speed_R_ble = sig_m2.spe * 100; // radian

  ////*** Totally, we send 32byte data
  // 0    header 165
  // 1    header 90
  // 2    bluetooth data length
  // ...

  data_ble[0] = 165;
  data_ble[1] = 90;
  data_ble[2] = datalength_ble;
  data_ble[3] = LK_ble;
  data_ble[4] = LK_ble >> 8;
  data_ble[5] = RK_ble;
  data_ble[6] = RK_ble >> 8;
  data_ble[7] = current_command_L_ble;
  data_ble[8] = current_command_L_ble >> 8;
  data_ble[9] = current_command_R_ble;
  data_ble[10] = current_command_R_ble >> 8;
  data_ble[11] = torque_command_L_ble;
  data_ble[12] = torque_command_L_ble >> 8;
  data_ble[13] = torque_command_R_ble;
  data_ble[14] = torque_command_R_ble >> 8;
  data_ble[15] = torque_estimated_L_ble;
  data_ble[16] = torque_estimated_L_ble >> 8;
  data_ble[17] = torque_estimated_R_ble;
  data_ble[18] = torque_estimated_R_ble >> 8;
  data_ble[19] = torque_measured_L_ble;
  data_ble[20] = torque_measured_L_ble >> 8;
  data_ble[21] = imu_gait_percent_L_ble;
  data_ble[22] = imu_gait_percent_L_ble >> 8;
  data_ble[23] = imu_gait_percent_R_ble;
  data_ble[24] = imu_gait_percent_R_ble >> 8;
  data_ble[25] = motor_speed_L_ble;
  data_ble[26] = motor_speed_L_ble >> 8;
  data_ble[27] = motor_speed_R_ble;
  data_ble[28] = motor_speed_R_ble >> 8;
  data_ble[29] = torque_measured_R_ble;
  data_ble[30] = torque_measured_R_ble >> 8;
  // data_ble[31] = imu.STSSitThreshold - 20;
  //

  Serial5.write(data_ble, datalength_ble);
}


void sendMessage(int messageID, int* messageValueArray, int messageLength)
{
  ////*** Totally, we send 32byte data
  // 0    header 165
  // 1    header 95
  // 2    bluetooth data length
  // 3    Message ID
  // ID = 0  All normal
  // ID = 1  if SD card detected on Teensy
  // ID = 2  if logged data file is saved

  data_ble[0] = 165;
  data_ble[1] = 95;
  data_ble[2] = datalength_ble;
  data_ble[3] = messageID;
  data_ble[4] = messageLength;
  for (int i = 0; i < messageLength; i++)
  {
    data_ble[5 + i] = messageValueArray[i];
  }
  Serial5.write(data_ble, datalength_ble);
  // Serial.println("Message sent");
  // Serial.println((double)data_ble[1]);
}

void Compute_Torque_Commands()
{
  if (assist_mode == 1) //Sine controller (IMU)
  {
    mode = "Sine (IMU)";
    controller.computeGaitPhase(imu.LTAVx - imu.RTAVx, 0.0, 0.0, 0.0, 0.0);
    controller.computeSingleSineAssistanceProfile();
    p_des = 0; //dont change this
    v_des = 0; //dont change this
    kp = 0; //dont change this
    kd = 0; //dont change this
    torque_command_L = controller.sineAssistanceTotalTorqueLeft;
    torque_command_R = -controller.sineAssistanceTotalTorqueRight;
    Cur_command_L = torque_command_L / torque_constant_before_gear / Gear_ratio;
    Cur_command_R = torque_command_R / torque_constant_before_gear / Gear_ratio;
    // Serial.println(Cur_command_L);
  }
  else if (assist_mode == 2) //Samsung controller (IMU)
  {
    mode = "Samsung (IMU)";
    controller.computeGaitPhase(imu.LTAVx - imu.RTAVx, 0.0, 0.0, 0.0, 0.0);
    p_des = 0; //dont change this
    v_des = 0; //dont change this
    kp = 0; //dont change this
    kd = 0; //dont change this
    torque_command_L = imu.DOTC[0];
    torque_command_R = -imu.DOTC[1];
    Cur_command_L = torque_command_L / torque_constant_before_gear / Gear_ratio;
    Cur_command_R = torque_command_R / torque_constant_before_gear / Gear_ratio;
    
  }
  else if (assist_mode == 3) //Constant torque (for hardware debugging purpose)
  {
    mode = "Constant";
    p_des = 0; //dont change this
    v_des = 0; //dont change this
    kp = 0; //dont change this
    kd = 0; //dont change this
    torque_command_L = 0;
    torque_command_R = 0;
    Cur_command_L = torque_command_L / torque_constant_before_gear / Gear_ratio;
    Cur_command_R = torque_command_R / torque_constant_before_gear / Gear_ratio;
  }
  

  else if (assist_mode == 100)
  {
    mode = "Stop";
    p_des = 0; //dont change this
    v_des = 0; //dont change this
    kp = 0; //dont change this
    kd = 0; //dont change this
    torque_command_L = 0;
    torque_command_R = 0;
    Cur_command_L = torque_command_L / torque_constant_before_gear / Gear_ratio;
    Cur_command_R = torque_command_R / torque_constant_before_gear / Gear_ratio;
  }
}

void IMUSetup() {
  imu.INIT();   
  delay(1500);     
  imu.INIT_MEAN();     
}  

double clip_torque(double torque_command)
{
  float actual_command = 0.0;  
  actual_command = fminf(fmaxf(MIN_torque_command, torque_command), MAX_torque_command);     

  return actual_command;   
}

void initial_CAN() {
  Can3.begin();
  // Can3.setBaudRate(1000000);  
  Can3.setBaudRate(1000000);  
  delay(400);  
  Serial.println("Can bus setup done...");  
  delay(200);  
}   

float Sig_torque_control(float force_des, float dt_force_des, float force_t, float dt_force_t, float kp, float kd, float tau_ff)  
{
  float tor_cmd = 0;   

  tor_cmd = kp * (force_des - force_t) + kd * (dt_force_des - dt_force_t) + tau_ff; 

  return tor_cmd;   
}  

float Sig_motion_control(float pos_des, float vel_des, float pos_t, float vel_t, float kp, float kd, float tau_ff)  
{
  float pos_ctl_cmd = 0;   

  pos_ctl_cmd = kp * (pos_des - pos_t) + kd * (vel_des - vel_t) + tau_ff; 

  return pos_ctl_cmd;   
}  

void receive_mit_ctl_feedback() {
  if (Can3.read(msgR)) {
    Can3.read(msgR);  

    if (msgR.id == 0x008)     
    {
      if (msgR.buf[0] == 0x000)      
      {
        sig_m1.unpack_reply(msgR, initial_pos_1);      
      } 
    } 
  }
}   

void receive_torque_ctl_feedback() {
  if (Can3.read(msgR)) {
    Can3.read(msgR);      

    if (msgR.id == 0x009)  
    {
      sig_m1.unpack_pos_vel(msgR, initial_pos_1);       
    } 

    if (msgR.id == 0x01C) 
    {
      sig_m1.unpack_torque(msgR);    
      tau_t_1 = sig_m1.torque;          
    }  

    if (msgR.id == 0x029)  
    {
      sig_m2.unpack_pos_vel(msgR, initial_pos_2);       
    } 

    if (msgR.id == 0x03C)   
    {
      sig_m2.unpack_torque(msgR);         
      tau_t_2 = sig_m2.torque;           
    }  
  }
}   

void fakeIMU() {
  // IMU11 = 150.0 * sin(t / 5.0);
  // IMU22 = 150.0 * cos(t / 5.0);
  // IMU33 = 700.0 * sin(t / 5.0);
  // IMU44 = 700.0 * cos(t / 5.0);   
  IMU11 = 150.0 * sin(2 * M_PI * pos_fre * t); 
  IMU22 = 150.0 * cos(2 * M_PI * pos_fre * t);
  IMU33 = 700.0 * sin(2 * M_PI * pos_fre * t);
  IMU44 = 700.0 * cos(2 * M_PI * pos_fre * t);   

  L_IMUX_int = Serial_Com.float_to_uint(IMU11, -180, 180, 16);
  R_IMUX_int = Serial_Com.float_to_uint(IMU22, -180, 180, 16); 

  L_IMUV_int = Serial_Com.float_to_uint(IMU33, -800, 800, 16);
  R_IMUV_int = Serial_Com.float_to_uint(IMU44, -800, 800, 16);

  Send[0] = 0x31;
  Send[1] = 0x32;  
  Send[2] = L_IMUX_int >> 8;   
  Send[3] = L_IMUX_int & 0xFF;
  Send[4] = R_IMUX_int >> 8;
  Send[5] = R_IMUX_int & 0xFF;
  Send[6] = L_IMUV_int >> 8;
  Send[7] = L_IMUV_int & 0xFF;
  Send[8] = R_IMUV_int >> 8;
  Send[9] = R_IMUV_int & 0xFF;
  Send[10] = 0x33;
}

void RealIMU() {
  f_LTAVx = LTAVx.addSample(imu.LTAVx);  
  f_RTAVx = RTAVx.addSample(imu.RTAVx);  

  L_IMUX_int = Serial_Com.float_to_uint(imu.LTx, -180, 180, 16);  
  R_IMUX_int = Serial_Com.float_to_uint(imu.RTx, -180, 180, 16);  

  //  L_IMUV_int = Serial_Com.float_to_uint(imu.LTAVx, -800, 800, 16);
  //  R_IMUV_int = Serial_Com.float_to_uint(imu.RTAVx, -800, 800, 16);

  L_IMUV_int = Serial_Com.float_to_uint(f_LTAVx, -800, 800, 16);  
  R_IMUV_int = Serial_Com.float_to_uint(f_RTAVx, -800, 800, 16);
  
  Send[0] = 0x31;  
  Send[1] = 0x32;
  Send[2] = L_IMUX_int >> 8;
  Send[3] = L_IMUX_int & 0xFF;
  Send[4] = R_IMUX_int >> 8;
  Send[5] = R_IMUX_int & 0xFF;
  Send[6] = L_IMUV_int >> 8;
  Send[7] = L_IMUV_int & 0xFF;
  Send[8] = R_IMUV_int >> 8;
  Send[9] = R_IMUV_int & 0xFF;
  Send[10] = 0x33;  
}

void UseEncoder(){ 
  L_IMUX_int = Serial_Com.float_to_uint(sig_m1.pos*180/M_PI, -180, 180, 16);     
  R_IMUX_int = Serial_Com.float_to_uint(sig_m2.pos*180/M_PI, -180, 180, 16);     

  L_IMUV_int = Serial_Com.float_to_uint(sig_m1.spe*180/M_PI, -800, 800, 16);     
  R_IMUV_int = Serial_Com.float_to_uint(sig_m2.spe*180/M_PI, -800, 800, 16);        
  
  Send[0] = 0x31;  
  Send[1] = 0x32;  
  Send[2] = L_IMUX_int >> 8;
  Send[3] = L_IMUX_int & 0xFF;
  Send[4] = R_IMUX_int >> 8;
  Send[5] = R_IMUX_int & 0xFF;
  Send[6] = L_IMUV_int >> 8;
  Send[7] = L_IMUV_int & 0xFF;
  Send[8] = R_IMUV_int >> 8;
  Send[9] = R_IMUV_int & 0xFF;  
  Send[10] = 0x33;   
}

void SendIMUSerial()
{
  L_IMUX_int = Serial_Com.float_to_uint(l_leg_angle, -180, 180, 16);     
  R_IMUX_int = Serial_Com.float_to_uint(r_leg_angle, -180, 180, 16);     

  L_IMUV_int = Serial_Com.float_to_uint(l_leg_velocity, -800, 800, 16);     
  R_IMUV_int = Serial_Com.float_to_uint(r_leg_velocity, -800, 800, 16);         
  
  Send[0] = 0x31;  
  Send[1] = 0x32;  
  Send[2] = L_IMUX_int >> 8;
  Send[3] = L_IMUX_int & 0xFF;  
  Send[4] = R_IMUX_int >> 8;  
  Send[5] = R_IMUX_int & 0xFF;  
  Send[6] = L_IMUV_int >> 8;  
  Send[7] = L_IMUV_int & 0xFF;  
  Send[8] = R_IMUV_int >> 8;  
  Send[9] = R_IMUV_int & 0xFF;  
  Send[10] = 0x33;   
}

void RealIMU_Reset() {
  float reset_imu = 0;

  L_IMUX_int = Serial_Com.float_to_uint(reset_imu, -180, 180, 16);
  R_IMUX_int = Serial_Com.float_to_uint(reset_imu, -180, 180, 16);

  //  L_IMUV_int = Serial_Com.float_to_uint(imu.LTAVx, -800, 800, 16);
  //  R_IMUV_int = Serial_Com.float_to_uint(imu.RTAVx, -800, 800, 16);

  L_IMUV_int = Serial_Com.float_to_uint(reset_imu, -800, 800, 16);
  R_IMUV_int = Serial_Com.float_to_uint(reset_imu, -800, 800, 16);

  Send[0] = 0x31; 
  Send[1] = 0x32;
  Send[2] = L_IMUX_int >> 8;
  Send[3] = L_IMUX_int & 0xFF;
  Send[4] = R_IMUX_int >> 8;
  Send[5] = R_IMUX_int & 0xFF;
  Send[6] = L_IMUV_int >> 8;
  Send[7] = L_IMUV_int & 0xFF;
  Send[8] = R_IMUV_int >> 8;
  Send[9] = R_IMUV_int & 0xFF;
  Send[10] = 0x33;
}

void Wait(unsigned long delay_control) {
  unsigned long Time_start = micros();
  unsigned long Time_Delta = delay_control;
  unsigned long Time_Control = 0;

  do {
    Time_Control = micros() - Time_start;
  } while (Time_Control < Time_Delta);
}

// void Receive_ble_Data(){
//   if (Serial5.available() >= 20) {
//     // Read the incoming byte:
//     Serial5.readBytes(&data_rs232_rx[0], 20);

//     if (data_rs232_rx[0] == 165) { // Check the first byte

//       if (data_rs232_rx[1] == 90) { // Check the second byte

//         if (data_rs232_rx[2] == 20) { // Check the number of elemnts in the package

//           Rescaling_gain   = ((int16_t)(data_rs232_rx[3] | (data_rs232_rx[4] << 8))) / 100.0;
//           Flex_Assist_gain = ((int16_t)(data_rs232_rx[5] | (data_rs232_rx[6] << 8))) / 100.0;
//           Ext_Assist_gain  = ((int16_t)(data_rs232_rx[7] | (data_rs232_rx[8] << 8))) / 100.0;
//           Assist_delay_gain = data_rs232_rx[9];

//           Serial.print(" | ");
//           Serial.print(Rescaling_gain);
//           Serial.print(" | ");
//           Serial.print(Flex_Assist_gain);
//           Serial.print(" | ");
//           Serial.print(Ext_Assist_gain);
//           Serial.print(" | ");
//           Serial.print(Assist_delay_gain);
//           Serial.println(" | ");
//         }
//       }
//     }
//   }
// }

// void Transmit_ble_Data() {
//   t_teensy        = t * 100;
//   L_leg_IMU_angle = imu.LTx * 100;
//   R_leg_IMU_angle = imu.RTx * 100;
//   L_motor_torque  = sig_m1.torque * 100;
//   R_motor_torque  = sig_m2.torque * 100;
//   L_motor_torque_command = M1_torque_command *100;
//   R_motor_torque_command = M2_torque_command *100;

//   L_load_cell_torque = torque_sensor1.torque[0] * 100;  
//   R_load_cell_torque = torque_sensor1.torque[1] * 100;    

//   ////*** Totally, we send 32byte data
//   // 0    header 165
//   // 1    header 90
//   // 2    bluetooth data length
//   // ...

//   data_ble[0]  = 165;
//   data_ble[1]  = 90;
//   data_ble[2]  = datalength_ble;
//   data_ble[3]  = t_teensy;
//   data_ble[4]  = t_teensy >> 8;
//   data_ble[5]  = L_leg_IMU_angle;
//   data_ble[6]  = L_leg_IMU_angle >> 8;
//   data_ble[7]  = R_leg_IMU_angle;
//   data_ble[8]  = R_leg_IMU_angle >> 8;
//   data_ble[9]  = L_load_cell_torque;
//   data_ble[10] = L_load_cell_torque >> 8;  
//   data_ble[11] = R_load_cell_torque;  
//   data_ble[12] = R_load_cell_torque >> 8;  
//   data_ble[13] = L_motor_torque_command;
//   data_ble[14] = L_motor_torque_command >> 8;
//   data_ble[15] = R_motor_torque_command;
//   data_ble[16] = R_motor_torque_command >> 8;
//   data_ble[17] = 0;
//   data_ble[18] = 0 >> 8;
//   data_ble[19] = 0;
//   data_ble[20] = 0;
//   data_ble[21] = 0;
//   data_ble[22] = 0 >> 8;
//   data_ble[23] = 0;
//   data_ble[24] = 0 >> 8;
//   data_ble[25] = 0;
//   data_ble[26] = 0 >> 8;
//   data_ble[27] = 0;
//   data_ble[28] = 0 >> 8;

//   Serial5.write(data_ble, datalength_ble);
// }

double derivative(double dt, double derivative_prev[], double *actual_in_ptr, double *prev_in_ptr){
  int i;
  double diff = 0.0, diff_sum = 0.0;
  if (dt != 0.0){
    for (i = 0; i < 3; i++){
      diff_sum += derivative_prev[i];
    }
    diff = (diff_sum + (*actual_in_ptr - *prev_in_ptr) / dt) / (i + 1);
  } else 
    diff = derivative_prev[3];
  return diff;
}

void print_Data_Ivan() {
  Serial.print(t);
  Serial.print(" ");
  Serial.print(imu.LTx);
  Serial.print(" ");
  Serial.print(imu.LTAVx);
  Serial.print(" ");
  Serial.print(sig_m1.torque);
  Serial.print(" ");
  Serial.print(L_CMD_serial);
  //Serial.print(sig_m2.torque);
  Serial.print(" ");
  Serial.println(" ");
}  

void print_Data_Jimmy() {
  Serial.print(t);
  Serial.print("Please give "); 
  // Serial.print(GUI_K);    
  // Serial.print(" ");   
  // Serial.print(imu.LTAVx);  
  // Serial.print(" ");
  // Serial.print(sig_m1.torque); 
  // Serial.print(" ");
  // Serial.print(sig_m1.pos);  
  // Serial.print(" ");
  // Serial.print(sig_m1.spe);    
  // Serial.print(" ");
  // Serial.print(L_CMD_serial);  
  //Serial.print(sig_m2.torque);  
  Serial.print(" "); 
  Serial.println(" ");  
}

void print_Data() {
  //  Serial.print(-20);
  //  Serial.print(" ");
  //  Serial.print(20);
  //  Serial.print(" ");  
  //  Serial.print(imu.RTx);
  //  Serial.print(" ");
  //  Serial.print(f_RTAVx);
  //  Serial.print(" ");  
  //  Serial.print(IMU22);
  //  Serial.print(" ");
  //  Serial.print( t_i / 1000 );
  //  Serial.print(" ");
  //  Serial.print(imu.RTx / 5);
  //  Serial.print(" ");
  Serial.print(imu.RTAVx / 10);
  Serial.print(" ");
  //  Serial.print(M1_torque_command);
  //  Serial.print(" ");
  //Serial.print(R_CMD_serial);//Received by Python from serial usb (commanded by the NN)
  //Serial.print(" ");

  // Serial.print(imu.LTx / 5);
  // Serial.print(" ");
  Serial.print(imu.LTAVx / 10);
  Serial.print(" ");
  Serial.print(sig_m2.torque);//Why is the sign opposite to m1?
  Serial.print(" ");  
  //  Serial.print(-M2_torque_command); // The one we send to the motor after R-CMD-SERIAL IS RECEIVED. Should be same as R_CMD_serial, unless saturation
  //  Serial.print(" ");
  //Serial.print(-sig_m2.torque);  //Feedback torque from the motor (estimated with current)
  Serial.print(M2_torque_command);
  Serial.print(" ");

  Serial.print(sig_m1.torque);
  Serial.print(" ");
  Serial.print(M1_torque_command);
  Serial.print(" ");
  Serial.print(LimitInf);
  Serial.print(" ");
  Serial.print(LimitSup);
  Serial.print(" ");   

  Serial.println(" ");
}

void print_Data_IMU() {
  Serial.print(-180);
  Serial.print(" ");
  Serial.print(180);
  Serial.print(" ");
  //  Serial.print(IMU22);
  Serial.print(" ");
  Serial.print(imu.LTx);
  Serial.print(" ");
  Serial.print(imu.LTAVx);
  Serial.print(" ");
  Serial.print(imu.RTx);
  Serial.print(" ");
  Serial.print(imu.RTAVx);
  Serial.println(" ");
}

void print_Data_Received() {
  Serial.print(20);
  Serial.print(" ");
  Serial.print(-20);
  Serial.print(" ");
  Serial.print(L_CMD_serial);
  Serial.print(" ");
  Serial.print(R_CMD_serial);
  Serial.print(" ");
  Serial.println(" ");  
}

void print_data_motor() {
  //  double v1 = 90;
  //  double v2 = -v1;
  //  Serial.print(v1);
  //  Serial.print("   ");
  //Serial.print(current_time);
  Serial.print(" ; ");
  Serial.print(" M1_tor ; "); //M1 is left, M2 is right
  Serial.print(sig_m1.torque);    
  Serial.print(" ; M1_cmd ; ");   
  Serial.print(M1_torque_command);   
  Serial.print(" ; M2_tor ; ");  
  Serial.print(sig_m2.torque);  
  Serial.print(" ; M2_cmd ; ");   
  Serial.print(M2_torque_command);
  Serial.print(" ; M1_pos ; ");
  Serial.print(sig_m1.pos);
  Serial.println(" ;  ");
}  

void M1_Torque_Impedance_Control_Example(){
    // p_des = l_pos_des;  //dont change this
    // v_des = l_vel_des;  //dont change this
    // kp = kp_imp;        //dont change this
    // kd = kd_imp;        //dont change this  
    // t_ff = 0.0;   

    p_des = 0.0;  //dont change this
    v_des = 0.0;  //dont change this
    kp = 0.0;        //dont change this
    kd = 0.0;        //dont change this  

    t_ff = M1_torque_command;   
    tau_imp = (p_des - sig_m1.pos) * kp + (v_des - sig_m1.spe) * kd + t_ff;    

    tau_imp = t_ff; 
    sig_m1.sig_mit_ctl_cmd(p_des, v_des, kp, kd, tau_imp);   

    receive_mit_ctl_feedback();  
} 

void Sig_M_Torque_Impedance_Control_Example(){
    // p_des = l_pos_des;  //dont change this
    // v_des = l_vel_des;  //dont change this
    // kp = kp_imp;        //dont change this
    // kd = kd_imp;        //dont change this  
    // t_ff = 0.0;   

    p_des = 0.0;  //dont change this    
    v_des = 0.0;  //dont change this    
    kp = 0.0;     //dont change this    
    kd = 0.0;     //dont change this     

    t_ff = M1_torque_command;   
    // tau_imp = (p_des - sig_m1.pos) * kp + (v_des - sig_m1.spe) * kd + t_ff;    

    tau_imp = t_ff;   
    sig_m1.sig_mit_ctl_cmd(p_des, v_des, kp, kd, tau_imp);    

    receive_mit_ctl_feedback();    
    // receive_mit_ctl_feedbcak();  
} 

void M2_Torque_Impedance_Control_Example(){
    p_des = 0.0;      //dont change this
    v_des = 0.0;  //dont change this
    kp = 0.0;        //dont change this
    kd = 0.0;        //dont change this  

    // t_ff = M1_torque_command;   
    t_ff = 0.3;    
    // tau_imp = (p_des - sig_m1.pos) * kp + (v_des - sig_m1.spe) * kd + t_ff;   
    tau_imp = t_ff;    
    sig_m2.sig_mit_ctl_cmd(p_des, v_des, kp, kd, tau_imp);    

    receive_mit_ctl_feedback();    
}  

void M1_Position_Control_Example() {
  position_command = 0;
  p_des = position_command * PI / 180;
  v_des = 0;  //dont change this
  kp = 30;    //max 450 min 0
  kd = 1.5;   //max 5 min 0
  t_ff = 0;   //dont change this
  sig_m1.sig_mit_ctl_cmd(p_des, v_des, kp, kd, t_ff);
  receive_mit_ctl_feedback();
}

void M2_Position_Control_Example() {
  position_command = 0;
  p_des = position_command * PI / 180;
  v_des = 0;  //dont change this
  kp = 30;    //max 450 min 0
  kd = 1.5;   //max 5 min 0
  t_ff = 0;   //dont change this
  sig_m2.sig_mit_ctl_cmd(p_des, v_des, kp, kd, t_ff);
  receive_mit_ctl_feedback();
}  

void SDCardSetup()
{
  sd.remove(LOG_FILENAME);

  // Initialize the SD.
  if (!sd.begin(SD_CONFIG))
  {
    sd.initErrorHalt(&Serial);
  }
  // Open or create file - truncate existing file.
  if (!file.open(LOG_FILENAME, O_RDWR | O_CREAT | O_TRUNC))
  {
    Serial.println("open failed\n");
    return;
  }
  // File must be pre-allocated to avoid huge
  // delays searching for free clusters.
  if (!file.preAllocate(LOG_FILE_SIZE))
  {
    Serial.println("preAllocate failed\n");
    file.close();
    return;
  }
  // initialize the RingBuf.
  rb.begin(&file);
  Serial.println("Data logging initialized.");
}

void SDCardSetup(const char* logFileName)
{
  sd.remove(logFileName);

  // Initialize the SD.
  if (!sd.begin(SD_CONFIG))
  {
    sd.initErrorHalt(&Serial);
  }
  // Open or create file - truncate existing file.
  if (!file.open(logFileName, O_RDWR | O_CREAT | O_TRUNC))
  {
    Serial.println("open failed\n");
    return;
  }
  // File must be pre-allocated to avoid huge
  // delays searching for free clusters.
  if (!file.preAllocate(LOG_FILE_SIZE))
  {
    Serial.println("preAllocate failed\n");
    file.close();
    return;
  }
  // initialize the RingBuf.
  rb.begin(&file);
  Serial.print("Data logging initialized. File name = ");
  Serial.println(logFileName);
}

void SDCardSaveToFile()
{
  rb.sync();
  file.truncate();
  file.rewind();
  file.close();
}  

void logdata()
{
  // Max RingBuf used bytes. Useful to understand RingBuf overrun.
  size_t maxUsed = 0;

  // Min spare micros in loop.
  int32_t minSpareMicros = INT32_MAX; 

  // Start time.
  uint32_t logTime = micros();
  // Log data until Serial input or file full.
  //  while (!Serial.available()) {
  // Amount of data in ringBuf.
  size_t n = rb.bytesUsed();
  if ((n + file.curPosition()) > (LOG_FILE_SIZE - 20))
  {
    Serial.println("File full - quitting.");
    return; // break;
  }
  if (n > maxUsed)  
  {
    maxUsed = n;
  }
  if (n >= 512 && !file.isBusy())  
  {
    // Not busy only allows one sector before possible busy wait.
    // Write one sector from RingBuf to file.
    if (512 != rb.writeOut(512))
    {
      Serial.println("writeOut failed");
      return; //break;
    }
  }

  rb.print(t);  
  rb.write(" ");   
  rb.print(torque_sensor1.torque[0]);     
  rb.write(" ");  
  rb.print(torque_sensor1.torque[1]);  
  rb.write(" ");
  rb.print(LTx_filtered);  
  rb.write(" ");
  rb.print(RTx_filtered);      

  if (rb.getWriteError())
  {
    // Error caused by too few free bytes in RingBuf.
    Serial.println("WriteError");
    return; //break;
  }
}  