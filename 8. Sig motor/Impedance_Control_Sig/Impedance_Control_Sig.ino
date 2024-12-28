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
// #include "ads1292r.h"   

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;   
 
/*Filter*/
MovingAverage LTAVx(12);  
MovingAverage RTAVx(12);    
float f_LTAVx = 0;  
float f_RTAVx = 0;  

CAN_message_t msgR;   
int CAN_ID = 3;   // Teensy 4.1 CAN bus port
int Sig_Motor_ID_1 = 0;     
int Sig_Motor_ID_2 = 1;       

double torque_command = 0;
double velocity_command = 0;  
double position_command = 0;  

double M1_torque_command = 0;
double M2_torque_command = 0;

double MAX_torque_command = 5;  
double MIN_torque_command = -5;  

int LimitInf = -18;
int LimitSup = 18;

float p_des = 0;   
float v_des = 0;    
float kp = 0;   
float kd = 0;   
float t_ff = 0;   

float initial_pos = 0;    

Motor_Control_Tmotor sig_m1(Sig_Motor_ID_1, CAN_ID);   
Motor_Control_Tmotor sig_m2(Sig_Motor_ID_2, CAN_ID);   

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
float L_CMD_serial = 0.0;

uint16_t R_CMD_int16 = 0x7fff;
float R_CMD_serial = 0.0;

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
// double HZ = 1.0 / (Delta_T1 / 1000.0);


//***For managing the Controller and Bluetooth rate
unsigned long t_0 = 0;
// double cyclespersec_ctrl = 28;  // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus) Controller must be faster than ble
double cyclespersec_ctrl = 100;  // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus) Controller must be faster than ble
double cyclespersec_ble  = 20;  // [Hz] Bluetooth sending data frequency
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
int L_leg_IMU_angle = 0;       //left knee angle
int R_leg_IMU = 0;  
int L_motor_torque = 0;   
int R_motor_torque = 0;  
int L_motor_torque_desired = 0;  
int R_motor_torque_desired = 0;  
int t_teensy = 0;  
int M_Selected = 0;  
int CtrlMode_Selected = 0;  

int GUI_force_cmd = 0;  
int GUI_stiffness_cmd = 0;  
int GUI_damping_cmd = 0;  

int GUI_pos_ampl_cmd    = 0;    
int GUI_pos_fre_cmd     = 0;    
int GUI_force_ampl_cmd  = 10;        
int GUI_force_fre_cmd   = 10;       

double GUI_force = 1.0;    
double GUI_K_p = 1.0;    
double GUI_K_d = 0.1;    

int L_pos_int_d = 0;    
int L_pos_int_a = 0;    
int L_vel_int_d = 0;      
int L_vel_int_a = 0;     
//**************************

//***Impedance Control Test***//  
double cmd_ampl = 1.0;       
double cmd_fre = 1.0;      

float pos_ampl = 0.2;   
float pos_fre = 0.5;   

float l_pos_des = 0.0;    
float l_vel_des = 0.0;    
float r_pos_des = 0.0;     
float r_vel_des = 0.0;     

float tau_imp = 0.0;      
float kp_imp = 1.0;     
float kd_imp = 0.01 * kp_imp;         
//***Impedance Control Test***//    

//***Torque Control Test */
float ref_force_ampl = 0.2;    
float ref_force_fre = 0.5;    

float tau_t = 0.0;  
float tau_t_last = 0.0;    
float tau_dt = 0.0;    
float dt = 0.01;   
float tau_ff = 0.0;  

float l_ref_tau = 0.0;   
float l_ref_tau_dt = 0.0;    
float r_ref_tau = 0.0;     
float r_ref_tau_dt = 0.0;     

//*** Motor Mode Set ***// 
int ctl_on = 1;    
int ctl_mode = 0;    // 0 for torque control, 1 for mit control   
int ctl_type = 0;    // 0 for motion, 1 for force tracking, 2 for direct torque   
//*** Motor Mode Set ***//    

void setup() {
  delay(3000); 
  Serial.begin(115200);   //115200/9600=12
  Serial2.begin(115200);  //115200/9600=12
  //Serial7.begin(115200);  // Communication with Raspberry PI or PC for High-lever controllers like RL
  Serial5.begin(115200);  //used to communication with bluetooth peripheral. Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)
  
  Serial_Com.INIT();  

  //#################
  Serial.println("SETUP DONE");
  Serial.print("Controller executed at ");
  Serial.print(String(cyclespersec_ctrl));
  Serial.println(" Hz");
  Serial.print("BT com executed at ");
  Serial.print(String(cyclespersec_ble));
  Serial.println(" Hz");
  //####################

  initial_CAN();    

  initial_Sig_motor();     

  // delay(100);  
  IMUSetup();  

  t_0 = micros();    
}

void initial_Sig_motor() {  
  // sig_m1.error_clear();    
  // delay(200); 

  // sig_m1.reboot();  
  // delay(200);  

  sig_m1.sig_motor_reset();   
  delay(1000);  
  
  sig_m1.sig_encoder_reset();    
  delay(10000);  

  /////////// set control mode /////////  
  if (ctl_mode == 1)  
  {
    sig_m1.sig_mit_ctl_mode_start();      
  } 
  else
  {
    sig_m1.sig_torque_ctl_mode_start();     
  } 
  // delay(200);    

  sig_m1.sig_motor_start();    
  // delay(200);    
  
  sig_m1.request_pos_vel();     
  // delay(200);  

  if (ctl_mode == 1)  
  {
    sig_m1.sig_mit_ctl_cmd(0.0, 0.0, 0.0, 0.0, 0.01);        
    receive_mit_ctl_data();    
  }
  else{
    sig_m1.request_torque();   
    sig_m1.sig_torque_cmd(0.01);      
    receive_torque_ctl_data();     
  }  
  // delay(100);     

  initial_pos = sig_m1.pos;    
  position_command = 0.0;   
  // M1_torque_command = 0.0;    
}

//// @brief ////
//// Zhimin Hou for Sig Motor ///  
void loop() {
    current_time = micros() - t_0;     
    t = current_time/1000000.0;      
    
    if (current_time - previous_time_ble > Tinterval_ble_micros) {

      Receive_ble_Data();     
      Transmit_ble_Data();      

      previous_time_ble = current_time;   
    }

    M1_torque_command = 0.0;       

    if (ctl_type == 0)
    {
      // reference position 
      l_pos_des = pos_ampl * sin(2 * M_PI * pos_fre * t);                                  // rad
      r_pos_des = pos_ampl * sin(2 * M_PI * pos_fre * t);                                  // rad      

      kp_imp = GUI_K_p;    
      kd_imp = 0.01 * GUI_K_p;     
      M1_torque_command = (l_pos_des - sig_m1.pos) * kp_imp + (0 - sig_m1.spe) * kd_imp;      
    } 
    else if (ctl_type == 1)   
    { 
      // Reference force  
      l_ref_tau = ref_force_ampl * sin(2 * M_PI * ref_force_fre  * t);      
      l_ref_tau_dt = ref_force_ampl * 2 * M_PI * ref_force_fre * cos(2 * M_PI * ref_force_fre * t);     

      r_ref_tau = ref_force_ampl * sin(2 * M_PI * ref_force_fre  * t);      
      r_ref_tau_dt = ref_force_ampl * 2 * M_PI * ref_force_fre * cos(2 * M_PI * ref_force_fre * t);   

      kp_imp = GUI_K_p;     
      kd_imp = 0.01 * GUI_K_p;     
      tau_ff = 0.0;    
      // M1_torque_command = Sig_torque_control(
      //   ref_tau, ref_tau_dt, 
      //   tau_t, tau_dt, 
      //   kp_imp, kd_imp, 
      //   tau_ff  
      // );    
    } 
    else
    {
      M1_torque_command = cmd_ampl * sin(2 * M_PI * cmd_fre * t);                         // Nm 
      // M1_torque_command = (l_pos_des - sig_m1.pos) * kp_imp + (0 - sig_m1.spe) * kd_imp;    // Nm
      // ref_tau = cmd_ampl * sin(2 * M_PI * 1/cmd_fre * t);     
      
      // forward torque control 
      // kp_imp = GUI_K_p;    
      // kd_imp = 0.01 * GUI_K_p;    
      // M1_torque_command = cmd_ampl * sin(2 * M_PI * 1/cmd_fre * t); 

      l_pos_des = pos_ampl * sin(2 * M_PI * pos_fre * t);                                  // rad
      r_pos_des = pos_ampl * sin(2 * M_PI * pos_fre * t);        

      M1_torque_command = 0.0;  
      M2_torque_command = 0.0; 
    } 

    // clip the torque command
    M1_torque_command = clip_torque(M1_torque_command);      
    M2_torque_command = clip_torque(M1_torque_command);      
  
    if (ctl_mode == 1)   
    {
      // torque control    
      sig_m1.sig_mit_ctl_cmd(0.0, 0.0, 10.0, 0.01, M1_torque_command * 1.0);      
      receive_mit_ctl_data();     
    } 
    else
    {
      sig_m1.request_torque();   
      receive_torque_ctl_data();     

      tau_t = sig_m1.torque;   
      
      next_t = micros() - t_0;   
      delta_t = next_t/1000000.0 - t;    

      tau_dt = (tau_t - tau_t_last)/delta_t;   
      tau_t_last = tau_t;   

      sig_m1.sig_torque_cmd(M1_torque_command);       
    }

    // print_Data_Jimmy();    
    // Wait(100);   
}  

void IMUSetup() {
  imu.INIT();
  delay(500);
  imu.INIT_MEAN();  
}

double clip_torque(double torque_command)
{
  float actual_command = 0.0;  
  actual_command = fminf(fmaxf(MIN_torque_command, torque_command), MAX_torque_command);     

  return actual_command;   
}

// void TorqueSensorSetup() {
//   torque_sensor1.Torque_sensor_initial();   //initial the torque sensor see ads1292r.cpp.//FOR THE TORQUE SENSORS
//   torque_sensor1.Torque_sensor_gain(0.0003446 * (-1) *1.97, 0.0003446 * (-1) * 1.8);   //set the calibration gain for torque sensor. Torque= gain* ADCvalue+offset.see ads1292r.cpp.//FOR THE TORQUE SENSORS. M1 Left (tape closer to header) - The first in this list.
//   torque_sensor1.Torque_sensor_offset_calibration();    //FOR THE TORQUE SENSORS
// }

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

    receive_mit_ctl_data();  
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

    receive_mit_ctl_data();    
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

    receive_mit_ctl_data();    
}  

void M1_Position_Control_Example() {
  position_command = 0;
  p_des = position_command * PI / 180;
  v_des = 0;  //dont change this
  kp = 30;    //max 450 min 0
  kd = 1.5;   //max 5 min 0
  t_ff = 0;   //dont change this
  sig_m1.sig_mit_ctl_cmd(p_des, v_des, kp, kd, t_ff);
  receive_mit_ctl_data();
}

void M2_Position_Control_Example() {
  position_command = 0;
  p_des = position_command * PI / 180;
  v_des = 0;  //dont change this
  kp = 30;    //max 450 min 0
  kd = 1.5;   //max 5 min 0
  t_ff = 0;   //dont change this
  sig_m2.sig_mit_ctl_cmd(p_des, v_des, kp, kd, t_ff);
  receive_mit_ctl_data();
}

void receive_mit_ctl_data() {
  if (Can3.read(msgR)) {
    Can3.read(msgR);  

    // int id = msgR.buf[0];     
    if (msgR.id == 0x008)     
    {
      if (msgR.buf[0] == 0x000)      
      {
        sig_m1.unpack_reply(msgR, initial_pos);      
      } 
    } 
  }
}   

void receive_torque_ctl_data() {
  if (Can3.read(msgR)) {
    Can3.read(msgR);  

    // int id = msgR.buf[0];   

    // if (id == Sig_Motor_ID_1)   
    // {
    //   sig_m1.unpack_reply(msgR);    
    // }  

    sig_m1.unpack_pos_vel(msgR, initial_pos);     
    sig_m1.unpack_torque(msgR);    
  }
}   

void fakeIMU() {
  IMU11 = 150.0 * sin(t / 5.0);
  IMU22 = 150.0 * cos(t / 5.0);
  IMU33 = 700.0 * sin(t / 5.0);
  IMU44 = 700.0 * cos(t / 5.0);

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

          GUI_stiffness_cmd = data_rs232_rx[8];   
          GUI_damping_cmd   = data_rs232_rx[9];    
          GUI_force_cmd     = data_rs232_rx[10];    

          GUI_pos_ampl_cmd    = data_rs232_rx[11];    
          GUI_pos_fre_cmd     = data_rs232_rx[12];    
          GUI_force_ampl_cmd  = data_rs232_rx[13];      
          GUI_force_fre_cmd   = data_rs232_rx[14];      

          GUI_K_p   = GUI_stiffness_cmd/value_scale;       
          GUI_K_d   = GUI_damping_cmd/value_scale;     
          GUI_force = GUI_force_cmd/value_scale;    

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
  L_leg_IMU_angle = imu.LTx * value_scale_ratio;    
  R_leg_IMU = imu.RTx * value_scale_ratio;   

  L_motor_torque = sig_m1.torque * value_scale_ratio;    
  // L_motor_torque_desired = l_ref_tau * 100;    
  L_motor_torque_desired = M1_torque_command * value_scale_ratio;    

  R_motor_torque = tau_dt * value_scale_ratio;     
  R_motor_torque_desired = tau_dt * value_scale_ratio;      

  L_pos_int_d = l_pos_des * 100;           
  L_pos_int_a = sig_m1.pos * 100;         

  L_vel_int_d = sig_m1.spe * value_scale_ratio;      
  L_vel_int_a = sig_m1.spe * value_scale_ratio;       

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
  data_ble[5]  = R_leg_IMU;  
  data_ble[6]  = R_leg_IMU >> 8;  
  data_ble[7]  = L_motor_torque; 
  data_ble[8]  = L_motor_torque >> 8; 
  data_ble[9]  = R_motor_torque; 
  data_ble[10] = R_motor_torque >> 8;
  data_ble[11] = L_motor_torque_desired;
  data_ble[12] = L_motor_torque_desired >> 8;
  data_ble[13] = R_motor_torque_desired;
  data_ble[14] = R_motor_torque_desired >> 8;
  data_ble[15] = t_teensy; 
  data_ble[16] = t_teensy >> 8;  
  data_ble[17] = L_pos_int_d;  
  data_ble[18] = L_pos_int_d >> 8;  
  data_ble[19] = L_pos_int_a;  
  data_ble[20] = L_pos_int_a >> 8;  
  data_ble[21] = 0;
  data_ble[22] = 0 >> 8;
  data_ble[23] = 0;
  data_ble[24] = 0 >> 8;
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
  Serial.print(" "); 
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


//float mAvg(float x){
//
//  /* Begin
//  #include "MovingAverage.h"
//  MovingAverage filter(12);
//  */
//
//  float avg=filter.addSample(x);
//  return avg;
//  }