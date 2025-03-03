//To make this work: Turn on motors. Load to Teensy while subject standing still. When done (data updating in Serial Monitor), run Python code ISRA_Main.py.
//Once that one is running (data updating in Command Window), start running. To change peak intensity, change lines 85-86 in the Python code.
#include "Serial_Com.h"
#include "Wireless_IMU.h"
#include <Arduino.h>
#include "MovingAverage.h"
#include <math.h>  
#include <iomanip> 
#include <cstring> 
#include <ArduinoEigenDense.h>

using namespace Eigen;

/*MOTOR*/ 
#include <FlexCAN_T4.h>   
#include "Sig_Motor_Control.h"   

// #include "ads1292r.h"   

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;   
 
/*Filter*/
MovingAverage LTAVx(12);    
MovingAverage RTAVx(12);    

MovingAverage LENCODER(12); 
MovingAverage RENCODER(12); 

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

double RL_torque_command_left = 0.0;    
double RL_torque_command_right = 0.0;    

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
//////////////////////////
uint16_t L_IMUX_int = 0x00;  
uint16_t R_IMUX_int = 0x00;   
uint16_t L_IMUV_int = 0x00;
uint16_t R_IMUV_int = 0x00;   
///////////////////////////

///////////////////////////
uint16_t L_CMD_int16 = 0x7fff;  
float L_CMD_serial   = 0.0;   
uint16_t R_CMD_int16 = 0x7fff;
float R_CMD_serial   = 0.0;  
///////////////////////////

///////////////////////////
uint16_t L_POS_int16 = 0x7fff;   
uint16_t R_POS_int16 = 0x7fff;  
uint16_t L_VEL_int16 = 0x7fff;   
uint16_t R_VEL_int16 = 0x7fff;    

// float R_POS_serial   = 0.0;  
// float L_POS_serial   = 0.0;   
// float R_POS_serial   = 0.0;  
// float L_POS_serial   = 0.0;    
///////////////////////////

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

double cmd_ampl = 0.0;        
double cmd_fre  = 1.0;      
float pos_ampl  = 0.0;      
float pos_fre   = 0.5;      

float ref_force_ampl = 0.2;    
float ref_force_fre  = 0.5;    

float l_ref_tau    = 0.0;   
float l_ref_tau_dt = 0.0;    
float r_ref_tau    = 0.0;     
float r_ref_tau_dt = 0.0;    

float l_leg_actual_pos = 0.0;  
float r_leg_actual_pos = 0.0;  
float l_leg_actual_vel = 0.0;   
float r_leg_actual_vel = 0.0;  
float l_leg_actual_vel_t = 0.0;   
float r_leg_actual_vel_t = 0.0;   

float l_leg_ref_pos  = 0.0; 
float r_leg_ref_pos  = 0.0;   
float l_leg_ref_vel  = 0.0;  
float r_leg_ref_vel  = 0.0;    

float l_leg_pred_pos = 0.0; 
float r_leg_pred_pos = 0.0; 

float tau_ff_l = 0.0;  
float tau_ff_r = 0.0; 

float torque_scale = 20.0; 
float position_scale = 180.0; 
float velocity_scale = 180.0;  

//***Impedance Control Test***//  
float tau_imp = 0.0;      
float kp_imp = 1.0;     
float kd_imp = 0.01 * kp_imp;         
//***Impedance Control Test***//    

//***Torque Control Test */
float dt = 0.01;    

float tau_t_1      = 0.0;  
float tau_t_1_last = 0.0;    
float tau_t_2      = 0.0;    
float tau_t_2_last = 0.0;   

float tau_dt_1  = 0.0;    
float tau_dt_2  = 0.0;     

float tau_ff_1  = 0.0;      
float tau_ff_2  = 0.0;   

//*** Motor Mode Set ***//   
int ctl_method  = 0;      // 0 for using RL controller, 1 for using other normal controller, 2 for this controller  
int ctl_mode    = 0;      // 0 for torque control, 1 for mit control    
int ctl_type    = 2;      // 0 for motion, 1 for force tracking, 2 for direct torque   

int sensor_type = 0;      // 0 for using IMU, 1 for using encoder   
int l_ctl_dir   = -1;     
int r_ctl_dir   = 1;    

int ref_type    = 0;      // 0 for torque 1 for reference motion 
//*** Motor Mode Set ***//    

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
  
  if (ref_type==1)
  {
    // /// reference motion  ///  
    Serial_Com.READREF();        
  }
  else
  {
    /// reference torque  ///  
    Serial_Com.READ2();       
  }   

  current_time = micros() - t_0;            
  t = current_time/1000000.0;          
  
  if (current_time - previous_time > Tinterval_ctrl_micros) 
  {
    if (current_time - previous_time_ble > Tinterval_ble_micros) {

      Receive_ble_Data();  
      Transmit_ble_Data();      
      previous_time_ble = current_time;   
    }  

    // kp_imp = GUI_K_p;        
    // kd_imp = 0.01 * GUI_K_p;           

    /// RL controller ///   
    if (ctl_method == 0)   
    {
      if (sensor_type == 0)      
      {
        //// use position and velocity from IMU  
        // RealIMU();   

        f_LTAVx = LTAVx.addSample(imu.LTAVx);  
        f_RTAVx = RTAVx.addSample(imu.RTAVx);   

        l_leg_actual_pos    = imu.LTx;     
        r_leg_actual_pos    = imu.RTx;     
        l_leg_actual_vel    = f_LTAVx;     
        r_leg_actual_vel    = f_RTAVx;     

        // fakeIMU();  
        // l_leg_actual_pos    = IMU11;   
        // r_leg_actual_pos    = IMU22;   
        // l_leg_actual_vel    = IMU33;   
        // r_leg_actual_vel    = IMU44;   
      }
      else
      {
        //// use position and velocity from encoder ////   
        //// up is negative; down is positive 
        l_leg_actual_pos = sig_m2.pos * 180/M_PI * r_ctl_dir;   
        r_leg_actual_pos = sig_m1.pos * 180/M_PI * l_ctl_dir;      

        l_leg_actual_vel_t = sig_m2.spe * 180/M_PI * r_ctl_dir;       
        r_leg_actual_vel_t = sig_m1.spe * 180/M_PI * l_ctl_dir;        

        l_leg_actual_vel = LENCODER.addSample(l_leg_actual_vel_t);    
        r_leg_actual_vel = RENCODER.addSample(r_leg_actual_vel_t);     
      } 

      /// send data to Pi4  
      SendIMUSerial();   
      Serial_Com.WRITE(Send, Send_Length);     

      /// receive data from Pi4   
      if (ref_type==1)  
      {
        /// receive reference from Pi4 
        L_POS_int16 = (Serial_Com.SerialDataRef[3] << 8) | Serial_Com.SerialDataRef[4];  
        l_leg_ref_pos   = Serial_Com.uint_to_float(L_POS_int16, -1 * position_scale, position_scale, 16);   
        R_POS_int16 = (Serial_Com.SerialDataRef[5] << 8) | Serial_Com.SerialDataRef[6];    
        r_leg_ref_pos   = Serial_Com.uint_to_float(R_POS_int16, -1 * position_scale, position_scale, 16);   
        
        L_VEL_int16 = (Serial_Com.SerialDataRef[7] << 8) | Serial_Com.SerialDataRef[8];  
        l_leg_ref_vel   = Serial_Com.uint_to_float(L_VEL_int16, -1 * velocity_scale, velocity_scale, 16);   
        R_VEL_int16 = (Serial_Com.SerialDataRef[9] << 8) | Serial_Com.SerialDataRef[10];    
        r_leg_ref_vel   = Serial_Com.uint_to_float(R_VEL_int16, -1 * velocity_scale, velocity_scale, 16);   

        Interactive_control();   
      }
      else
      {
        L_CMD_int16 = (Serial_Com.SerialData2[3] << 8) | Serial_Com.SerialData2[4];    
        L_CMD_serial = Serial_Com.uint_to_float(L_CMD_int16, -20, 20, 16);    

        R_CMD_int16 = (Serial_Com.SerialData2[5] << 8) | Serial_Com.SerialData2[6];    
        R_CMD_serial = Serial_Com.uint_to_float(R_CMD_int16, -20, 20, 16);     

        RL_torque_command_left  = assistive_ratio * L_CMD_serial;       
        RL_torque_command_right = assistive_ratio * R_CMD_serial;       
      } 
      
      M1_torque_command = RL_torque_command_right * r_ctl_dir;        /// for right.   
      M2_torque_command = RL_torque_command_left * l_ctl_dir;         /// for left.     

      // M1_torque_command = 0.0 * r_ctl_dir;                         /// for right.  
      // M2_torque_command = 0.0 * l_ctl_dir;                         /// for left.    
    }
    else if (ctl_method == 1)  
    {
      /// other controllers ///
      if (ctl_type == 0)  
      {
        // reference position 
        l_leg_ref_pos = pos_ampl * sin(2 * M_PI * pos_fre * t);                            // rad
        r_leg_ref_pos = pos_ampl * sin(2 * M_PI * pos_fre * t);                            // rad      

        // reference velocity 
        l_leg_ref_vel = 2 * M_PI * pos_fre * pos_ampl * cos(2 * M_PI * pos_fre * t);       // rad/s                                
        r_leg_ref_vel = 2 * M_PI * pos_fre * pos_ampl * cos(2 * M_PI * pos_fre * t);       // rad/s    

        // // for impedance demo 
        // l_leg_ref_pos = 0.0;             
        // r_leg_ref_pos = 0.0;                     
        // l_leg_ref_vel = 0.0;              
        // r_leg_ref_vel = 0.0;             

        M1_torque_command = Sig_motion_control(
          l_leg_ref_pos, l_leg_ref_vel, 
          sig_m1.pos, sig_m1.spe, 
          kp_imp, kd_imp, 
          tau_ff_1
        ); 
        M2_torque_command = Sig_motion_control(
          r_leg_ref_pos, r_leg_ref_vel, 
          sig_m2.pos, sig_m2.spe, 
          kp_imp, kd_imp, 
          tau_ff_2
        );      
      } 
      else if (ctl_type == 1)   
      { 
        // Reference force  
        l_ref_tau = ref_force_ampl * sin(2 * M_PI * ref_force_fre  * t);      
        r_ref_tau = ref_force_ampl * sin(2 * M_PI * ref_force_fre  * t);      

        l_ref_tau_dt = ref_force_ampl * 2 * M_PI * ref_force_fre * cos(2 * M_PI * ref_force_fre * t);     
        r_ref_tau_dt = ref_force_ampl * 2 * M_PI * ref_force_fre * cos(2 * M_PI * ref_force_fre * t);   

        M1_torque_command = Sig_torque_control(
          l_ref_tau, l_ref_tau_dt, 
          tau_t_1, tau_dt_1,  
          kp_imp, kd_imp, 
          tau_ff_1  
        );    
        M2_torque_command = Sig_torque_control(
          r_ref_tau, r_ref_tau_dt, 
          tau_t_2, tau_dt_2,  
          kp_imp, kd_imp, 
          tau_ff_2  
        );    
      } 
      else
      {
        //// use position and velocity from encoder ////   
        // UseEncoder();  
        l_leg_actual_pos = sig_m2.pos * 180/M_PI * l_ctl_dir;   
        r_leg_actual_pos = sig_m1.pos * 180/M_PI * r_ctl_dir;      

        l_leg_actual_vel = sig_m2.spe * 180/M_PI * l_ctl_dir;    
        r_leg_actual_vel = sig_m1.spe * 180/M_PI * r_ctl_dir;    

        // direct torque  
        M1_torque_command = cmd_ampl * sin(2 * M_PI * cmd_fre * t);       
        M2_torque_command = cmd_ampl * sin(2 * M_PI * cmd_fre * t);                

        l_leg_ref_pos = pos_ampl * sin(2 * M_PI * pos_fre * t);                           
        r_leg_ref_pos = pos_ampl * sin(2 * M_PI * pos_fre * t);        

        // M1_torque_command = 0.0;     
        // M2_torque_command = 0.0;     
        Serial.print("Please give the exact control method!!!");     
      }  
    }  
    else if (ctl_method == 2)    
    {
      /* code */
    } 
    else{
      M1_torque_command = 0.0 * r_ctl_dir;                    /// for right. 
      M2_torque_command = 0.0 * l_ctl_dir;                    /// for left.  
    }  

    // clip the torque command for motors 
    M1_torque_command = clip_torque(M1_torque_command);         
    M2_torque_command = clip_torque(M2_torque_command);           
  
    // if (ctl_mode == 1)    
    // {
    //   // mit control    
    //   sig_m1.sig_mit_ctl_cmd(0.0, 0.0, 10.0, 0.01, M1_torque_command);      
    //   sig_m2.sig_mit_ctl_cmd(0.0, 0.0, 10.0, 0.01, M2_torque_command);       
    //   receive_mit_ctl_feedback();      
    // } 
    // else 
    // {
    // for (int i =0; i < 4; i++)   
    // {
    //   receive_torque_ctl_feedback();     
    // }  
    
    receive_torque_ctl_feedback();   
    // next_t = micros() - t_0;   
    // delta_t = next_t/1000000.0 - t;    

    // tau_dt_1 = (tau_t_1 - tau_t_1_last)/delta_t;     
    // tau_dt_2 = (tau_t_2 - tau_t_2_last)/delta_t;     
    
    // tau_t_1_last = tau_t_1;      
    // tau_t_2_last = tau_t_2;         

    sig_m1.sig_torque_cmd(M1_torque_command);       
    sig_m2.sig_torque_cmd(M2_torque_command);      
    // }

    // print_Data_Jimmy();     
    // Wait(2200);     

    previous_time = current_time;   
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

/// @brief interactive control ///
/// @return 
void Interactive_control()
{
  RL_torque_command_left  = (l_leg_actual_pos - l_leg_ref_pos) * kp_imp + (l_leg_actual_vel - l_leg_ref_vel) * kd_imp + tau_ff_l;  
  RL_torque_command_right = (r_leg_actual_pos - r_leg_ref_pos) * kp_imp + (r_leg_actual_vel - r_leg_ref_vel) * kd_imp + tau_ff_r;  
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
  // sig_m1.request_torque();       
  // sig_m2.request_torque();      
  sig_m1.request_pos_vel();   
  sig_m2.request_pos_vel();   
  if (Can3.read(msgR)) {
    // Can3.read(msgR);    
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
  L_IMUX_int = Serial_Com.float_to_uint(l_leg_actual_pos, -180, 180, 16);     
  R_IMUX_int = Serial_Com.float_to_uint(r_leg_actual_pos, -180, 180, 16);     

  L_IMUV_int = Serial_Com.float_to_uint(l_leg_actual_vel, -800, 800, 16);     
  R_IMUV_int = Serial_Com.float_to_uint(r_leg_actual_vel, -800, 800, 16);         
  
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

    float value_scale = 10.0;     
    if (data_rs232_rx[0] == 165) { // Check the first byte
      if (data_rs232_rx[1] == 90) { // Check the second byte
        if (data_rs232_rx[2] == 20) { // Check the number of elemnts in the package

          M_Selected        = data_rs232_rx[4];    
          CtrlMode_Selected = data_rs232_rx[6];    

          GUI_stiffness_cmd = data_rs232_rx[7];    
          GUI_damping_cmd   = data_rs232_rx[8];    
          GUI_force_cmd     = data_rs232_rx[9];    
          GUI_assistive_ratio_cmd = data_rs232_rx[10];     

          GUI_K_p   = GUI_stiffness_cmd/value_scale;       
          GUI_K_d   = GUI_damping_cmd/value_scale;     
          GUI_force = GUI_force_cmd/value_scale;    
          assistive_ratio = GUI_assistive_ratio_cmd/value_scale/10.0;    

          /// reference  
          GUI_pos_ampl_cmd    = data_rs232_rx[11];    
          GUI_pos_fre_cmd     = data_rs232_rx[12];    
          GUI_force_ampl_cmd  = data_rs232_rx[13];      
          GUI_force_fre_cmd   = data_rs232_rx[14];      

          pos_ampl = GUI_pos_ampl_cmd/value_scale;  
          pos_fre  = GUI_pos_fre_cmd/value_scale;  
          cmd_ampl = GUI_force_ampl_cmd/value_scale;    
          cmd_fre  = GUI_force_fre_cmd/value_scale;   

          Serial.print("| Motor ");
          Serial.print(M_Selected, DEC); // This contains the motor number
          Serial.print(" selected | Control Mode ");
          Serial.print(CtrlMode_Selected, DEC); // This contains the Control mode
          Serial.print(" selected | Command ");
          //Serial.print(GUI_force_cmd, DEC); // This contains the desired command
          // Serial.print(GUI_K);  
          Serial.print(GUI_K_p);  
          Serial.print(GUI_K_d);  
          Serial.println(" sent |");
        }
      }
    }
  }
}

void Transmit_ble_Data(){ 
  float value_scale_ratio = 100;   

  t_teensy = t * value_scale_ratio;      

  // L_leg_IMU_angle = imu.LTx * value_scale_ratio;       
  // R_leg_IMU_angle = imu.RTx * value_scale_ratio;      

  // L_leg_IMU_angle = sig_m1.pos*180/M_PI * value_scale_ratio;       
  // R_leg_IMU_angle = sig_m2.pos*180/M_PI * value_scale_ratio;      

  L_leg_IMU_angle = l_leg_actual_pos * value_scale_ratio;   
  R_leg_IMU_angle = r_leg_actual_pos * value_scale_ratio;    

  // L_motor_torque = sig_m2.torque * value_scale_ratio;    
  L_motor_torque = tau_t_1 * value_scale_ratio;     
  // L_motor_torque_desired = M1_torque_command * value_scale_ratio;     
  L_motor_torque_desired = RL_torque_command_left * value_scale_ratio;           

  // R_motor_torque = sig_m1.torque * value_scale_ratio;     
  R_motor_torque = tau_t_2 * value_scale_ratio;   
  // R_motor_torque_desired = M2_torque_command * value_scale_ratio;    
  // R_motor_torque_desired = sig_m1.torque * value_scale_ratio;     
  R_motor_torque_desired = RL_torque_command_right * value_scale_ratio;      

  // L_pos_int_d = l_leg_ref_pos * value_scale_ratio;   
  L_pos_int_d = l_leg_ref_vel * value_scale_ratio;                 
  // L_pos_int_a = sig_m2.pos * value_scale_ratio;              
  L_pos_int_a = l_leg_actual_pos * value_scale_ratio;          

  L_vel_int_d = l_leg_ref_vel * value_scale_ratio;      
  // L_vel_int_a = sig_m2.spe * value_scale_ratio;       
  L_vel_int_a = l_leg_actual_vel * value_scale_ratio;       

  R_pos_int_d = r_leg_ref_vel * value_scale_ratio;         
  // R_pos_int_d = r_leg_ref_pos * value_scale_ratio;                
  // R_pos_int_a = sig_m1.pos * value_scale_ratio;       
  R_pos_int_a = r_leg_actual_pos * value_scale_ratio;           

  R_vel_int_d = r_leg_ref_vel * value_scale_ratio;      
  // R_vel_int_a = sig_m1.spe * value_scale_ratio;    
  R_vel_int_a = r_leg_actual_vel * value_scale_ratio;            

  ////*** Totally, we send 32byte data
  // 0    header 165
  // 1    header 90
  // 2    bluetooth data length
  // ...   
  data_ble[0]  = 165;  
  data_ble[1]  = 90;  
  data_ble[2]  = datalength_ble;  
  data_ble[3]  = L_leg_IMU_angle;  
  data_ble[4]  = L_leg_IMU_angle >> 8;  
  data_ble[5]  = R_leg_IMU_angle;  
  data_ble[6]  = R_leg_IMU_angle >> 8;  
  data_ble[7]  = L_motor_torque; 
  data_ble[8]  = L_motor_torque >> 8; 
  data_ble[9]  = R_motor_torque;  
  data_ble[10] = R_motor_torque >> 8;   
  data_ble[11] = L_motor_torque_desired;  
  data_ble[12] = L_motor_torque_desired >> 8;   
  data_ble[13] = R_motor_torque_desired;   
  data_ble[14] = R_motor_torque_desired >> 8;    
  // data_ble[11] = RL_torque_command_left;  
  // data_ble[12] = RL_torque_command_left >> 8;
  // data_ble[13] = RL_torque_command_right;
  // data_ble[14] = RL_torque_command_right >> 8;    
  data_ble[15] = t_teensy; 
  data_ble[16] = t_teensy >> 8;  
  data_ble[17] = L_pos_int_d;  
  data_ble[18] = L_pos_int_d >> 8;  
  data_ble[19] = L_pos_int_a;  
  data_ble[20] = L_pos_int_a >> 8;  
  data_ble[21] = R_pos_int_d;  
  data_ble[22] = R_pos_int_d >> 8;  
  data_ble[23] = R_pos_int_a;  
  data_ble[24] = R_pos_int_a >> 8;  
  data_ble[25] = 0;
  data_ble[26] = 0 >> 8;
  data_ble[27] = 0;
  data_ble[28] = 0 >> 8;

  Serial5.write(data_ble, datalength_ble);  
  //Serial7.write(data_ble, datalength_ble);
  //Serial.print("Transmit Data Function Executed");
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
    // p_des = l_leg_ref_pos;  //dont change this
    // v_des = l_leg_ref_vel;  //dont change this
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
    // p_des = l_leg_ref_pos;  //dont change this
    // v_des = l_leg_ref_vel;  //dont change this
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

// void TorqueSensorSetup() {
//   torque_sensor1.Torque_sensor_initial();   //initial the torque sensor see ads1292r.cpp.//FOR THE TORQUE SENSORS
//   torque_sensor1.Torque_sensor_gain(0.0003446 * (-1) *1.97, 0.0003446 * (-1) * 1.8);   //set the calibration gain for torque sensor. Torque= gain* ADCvalue+offset.see ads1292r.cpp.//FOR THE TORQUE SENSORS. M1 Left (tape closer to header) - The first in this list.
//   torque_sensor1.Torque_sensor_offset_calibration();    //FOR THE TORQUE SENSORS
// }  