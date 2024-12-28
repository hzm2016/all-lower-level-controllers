//To make this work: Turn on motors. Load to Teensy while subject standing still. When done (data updating in Serial Monitor), run Python code ISRA_Main.py.
//Once that one is running (data updating in Command Window), start running. To change peak intensity, change lines 85-86 in the Python code.
#include "Serial_Com.h"
#include "Wireless_IMU.h"
#include <Arduino.h>
#include "MovingAverage.h"
#include <math.h>  
#include <iomanip> 
#include <cstring>  

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
#define LOG_FILENAME "2022-05-24-Weibo-Walking_Powered_06.csv"

SdFs sd;
FsFile file;  

// RingBuf for File type FsFile.
RingBuf<FsFile, RING_BUF_CAPACITY> rb;    

ads1292r torque_sensor1;                                      //Create torque sensor object see ads1292r.h

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
int CAN_ID = 3;  

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
    imu.READ();   
    Serial_Com.READ2();       

    current_time = micros() - t_0;            
    t = current_time/1000000.0;           
    
    if (current_time - previous_time > Tinterval_ctrl_micros) {
      if (current_time - previous_time_ble > Tinterval_ble_micros) {

        Receive_ble_Data();  
        Transmit_ble_Data();      
        previous_time_ble = current_time;   
      }  

      /// Samsung Controller ///  
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
        S_torque_command_left = - Rescaling_gain * Ext_Assist_gain * torque_delay[delayindex]; // left hip torque
        S_torque_command_right = Rescaling_gain * Flex_Assist_gain * torque_delay[delayindex];  // right hip torque
      }
      else if ((RLTx_delay[delayindex] < 0) && (RLTx_delay[delayindex] > -120)) {
        S_torque_command_left = Rescaling_gain * Ext_Assist_gain * torque_delay[delayindex];  // right hip torque
        S_torque_command_right = -Rescaling_gain * Flex_Assist_gain * torque_delay[delayindex]; // left hip torque
      }
      else {
        // M2_torque_command=0;  
        // M1_torque_command=0;   
      }

      int max_allowed_torque = 30; // Safety measurement to limit the commanded torque
      
      // clip the torque command  
      M1_torque_command = clip_torque(M1_torque_command);         
      M2_torque_command = clip_torque(M2_torque_command);           

      M1_torque_command = S_torque_command_right * r_ctl_dir;        /// for right.   
      M2_torque_command = S_torque_command_left * l_ctl_dir;   
      if (ctl_mode == 1)    
      {
        // mit control    
        sig_m1.sig_mit_ctl_cmd(0.0, 0.0, 10.0, 0.01, M1_torque_command);      
        sig_m2.sig_mit_ctl_cmd(0.0, 0.0, 10.0, 0.01, M2_torque_command);       
        receive_mit_ctl_feedback();      
      } 
      else 
      {
        // sig_m1.request_torque();       
        // sig_m2.request_torque();      

        for (int i =0; i < 4; i++)   
        {
          receive_torque_ctl_feedback();     
        }
        // next_t = micros() - t_0;   
        // delta_t = next_t/1000000.0 - t;    

        // tau_dt_1 = (tau_t_1 - tau_t_1_last)/delta_t;     
        // tau_dt_2 = (tau_t_2 - tau_t_2_last)/delta_t;     
        
        // tau_t_1_last = tau_t_1;      
        // tau_t_2_last = tau_t_2;         

        sig_m1.sig_torque_cmd(M1_torque_command);      
        sig_m2.sig_torque_cmd(M2_torque_command);        
      }

      torque_sensor1.Torque_sensor_read();   

      previous_time = current_time;   

      logdata();  
    }  

    SDCardSaveToFile(); 
}  

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

void Receive_ble_Data(){
  if (Serial5.available() >= 20) {
    // Read the incoming byte:
    Serial5.readBytes(&data_rs232_rx[0], 20);

    if (data_rs232_rx[0] == 165) { // Check the first byte

      if (data_rs232_rx[1] == 90) { // Check the second byte

        if (data_rs232_rx[2] == 20) { // Check the number of elemnts in the package

          Rescaling_gain   = ((int16_t)(data_rs232_rx[3] | (data_rs232_rx[4] << 8))) / 100.0;
          Flex_Assist_gain = ((int16_t)(data_rs232_rx[5] | (data_rs232_rx[6] << 8))) / 100.0;
          Ext_Assist_gain  = ((int16_t)(data_rs232_rx[7] | (data_rs232_rx[8] << 8))) / 100.0;
          Assist_delay_gain = data_rs232_rx[9];

          Serial.print(" | ");
          Serial.print(Rescaling_gain);
          Serial.print(" | ");
          Serial.print(Flex_Assist_gain);
          Serial.print(" | ");
          Serial.print(Ext_Assist_gain);
          Serial.print(" | ");
          Serial.print(Assist_delay_gain);
          Serial.println(" | ");
        }
      }
    }
  }
}

void Transmit_ble_Data() {
  t_teensy        = t * 100;
  L_leg_IMU_angle = imu.LTx * 100;
  R_leg_IMU_angle = imu.RTx * 100;
  L_motor_torque  = sig_m1.torque * 100;
  R_motor_torque  = sig_m2.torque * 100;
  L_motor_torque_command = M1_torque_command *100;
  R_motor_torque_command = M2_torque_command *100;

  L_load_cell_torque = torque_sensor1.torque[0] * 100;  
  R_load_cell_torque = torque_sensor1.torque[1] * 100;    

  ////*** Totally, we send 32byte data
  // 0    header 165
  // 1    header 90
  // 2    bluetooth data length
  // ...

  data_ble[0]  = 165;
  data_ble[1]  = 90;
  data_ble[2]  = datalength_ble;
  data_ble[3]  = t_teensy;
  data_ble[4]  = t_teensy >> 8;
  data_ble[5]  = L_leg_IMU_angle;
  data_ble[6]  = L_leg_IMU_angle >> 8;
  data_ble[7]  = R_leg_IMU_angle;
  data_ble[8]  = R_leg_IMU_angle >> 8;
  data_ble[9]  = L_load_cell_torque;
  data_ble[10] = L_load_cell_torque >> 8;  
  data_ble[11] = R_load_cell_torque;  
  data_ble[12] = R_load_cell_torque >> 8;  
  data_ble[13] = L_motor_torque_command;
  data_ble[14] = L_motor_torque_command >> 8;
  data_ble[15] = R_motor_torque_command;
  data_ble[16] = R_motor_torque_command >> 8;
  data_ble[17] = 0;
  data_ble[18] = 0 >> 8;
  data_ble[19] = 0;
  data_ble[20] = 0;
  data_ble[21] = 0;
  data_ble[22] = 0 >> 8;
  data_ble[23] = 0;
  data_ble[24] = 0 >> 8;
  data_ble[25] = 0;
  data_ble[26] = 0 >> 8;
  data_ble[27] = 0;
  data_ble[28] = 0 >> 8;

  Serial5.write(data_ble, datalength_ble);
}

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