/*************************************/
/*                                   */
/*          HAMR_embedded.ino        */
/*                MODLAB             */
/*                                   */
/*************************************/
// Notes:
// M1 => RIGHT MOTOR
// M2 => LEFT MOTOR
// MT => TURRET MOTOR
#include <Arduino_mbed.h>
#include <as5048spi.h>
#include <SPI.h>
#include <mbed.h>

//Rosserial_Arduino needed head files
#define USE_USBCON
#include <ros.h>
#include <std_msgs/Int32.h>
#include <BasePos.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include <constants.h>
#include <holonomic_control.h>
#include <message_types.h>
#include <message_manager.h>
#include <additional_serial.h>
#include <LpfButter1.h>
#include <RunningMedian.h>
#include <pid_linear.hpp>
#include <motor.h>

/***************************************************/
/*                                                 */
/*                   VARIABLES                     */
/*                                                 */
/***************************************************/

int debugmessage = 0;
int wifi_mode = 0; // 0 for AP mode, 1 for CL mode

/******************/
/* Desired Values */
/******************/
// holonomic velocities
float desired_h_xdot = 0;
float desired_h_ydot = 0;
float desired_h_rdot = 0;

// motor velocities
float desired_M1_v = 0;
float desired_M2_v = 0;
float desired_MT_v = 0;

/*****************/
/* Sensed Values */
/*****************/
// sensed motor velocities at a specific point in time
float sensed_M1_v = 0.0;
float sensed_M2_v = 0.0;
float sensed_MT_v = 0.0;

// Previously sensed velocities
// Used in the current implementation of the low-pass filter
float prev_desired_M1_v = 0.0;
float prev_desired_M2_v = 0.0;
float prev_desired_MT_v = 0.0;

/************************/
/*  Computed Values     */
/************************/
// Computed holonomic velocities
float computed_xdot = 0.0;
float computed_ydot = 0.0;
float computed_tdot = 0.0;

// Summation of turret
long decoder_turret_total = 0; // Be careful of overflow

/******************/
/* Set Speed Vars */
/******************/
// motor PWMs

int pwm_M1 = 0;
int pwm_M2 = 0;
int pwm_MT = 0;
float M1_v_cmd = 0;
float M2_v_cmd = 0;
float MT_v_cmd = 0;

//motor serials
int serial_M1 = 0;
int serial_M2 = 0;
int serial_MT = 0;

// motor tags
int turret_motor_tag = 1;
int left_motor_tag = 2;
int right_motor_tag = 3;

/**********************/
/* Control Parameters */
/**********************/
// Low-level PID
PidLinear speed_M1_pid;
PidLinear speed_M2_pid;
PidLinear speed_MT_pid;

//Butterworth filter for each motor. This value should be tuned.
LpfButter1 m1_filter_speed(10,LOOPHZ);
LpfButter1 m2_filter_speed(10,LOOPHZ);
LpfButter1 mt_filter_speed(10,LOOPHZ);

//Running Median 
RunningMedian samples_MT = RunningMedian(5);
RunningMedian samples_M1 = RunningMedian(5);
RunningMedian samples_M2 = RunningMedian(5);

//-----------------------------------------------
// Velocity control command
// NOTE: These vars could be deleted provided we're not doing PID stuff
float h_xdot_cmd = 0;
float h_ydot_cmd = 0;
float h_rdot_cmd = 0; //holonomic
float dtheta_cmd = 0; //differential drive

/**********************/
/*     Sensor         */
/**********************/
// decoder counts
int decoder_count_M1 = 0;
int decoder_count_M2 = 0;
int decoder_count_MT = 0;

// Previously recorded motor readings- used to find velocity
int decoder_count_M1_prev = 0;
int decoder_count_M2_prev = 0;
int decoder_count_MT_prev = 0;

// Previously recorded decoder change. used for error handling
int decoder_count_change_M1_prev = 0;
int decoder_count_change_M2_prev = 0;
int decoder_count_change_MT_prev = 0;

/**********************/
/*  IMU Settings      */
/**********************/
// This is not used because we don't have an IMU
const float SENSOR_LOOPTIME = 10; //10000 //remeber to update this to microseconds later, or an exact timing method if possible
unsigned long next_sensor_time = millis(); // micros();
unsigned long prev_micros = 0;
float sensed_drive_angle = 0;

/************************/
/*    Timing            */
/************************/
unsigned long start_time; // Time at which the arduino starts
unsigned long last_recorded_time = 0;
float time_elapsed_micros; // microseconds
float time_elapsed_millis; // time_elapsed converted
float time_elapsed_secs;   //time elapsed in seconds
float time_elapsed_secs_inv;   //time elapsed in seconds
int loop_time_duration; // Timing loop time- for performance testing
unsigned long last_debug_time = 0;//Timing tracker for debug messages.
unsigned long last_command_time = 0;//Timing tracker for command signals.
unsigned long last_time_4_section_debugger =0;
unsigned long last_update_msg_time  = 0;
int loop_time_error = 0;
unsigned int avg_time = 0;
int loop_times[3];

/******************************/
/*  Test Drive Variables      */
/******************************/
unsigned long start_test_time;
unsigned long current_time;
float seconds = 0;
float R = 0;
bool square_test_did_start = false;
bool right_test_did_start = false;
bool circle_test_did_start = false;
bool spiral_test_did_start = false;
bool sine_test_did_start = false;
bool heading_circle_test_did_start = false;
bool timer_set = false;

/***************************/
/*      Modes              */
/***************************/
// Use direct drive by sending false to both dif_drive and holonomic_drive
bool use_dif_drive = false; // Sending else linear velocity commands and turret commands
bool use_holonomic_drive = true; // Full fledged holonomic drive

/****************************************/
/*      Motor Calculations              */
/****************************************/
// Previously sensed velocities
float prev_sensed_velocity_right;
float prev_sensed_velocity_left;
float prev_sensed_velocity_turret;
As5048Spi angle_sensor_M2(A6, A5, A4, D7);
As5048Spi angle_sensor_M1(A6, A5, A4, A0);
As5048Spi angle_sensor_MT(A6, A5, A4, D6);

/***********************/
/*  Miscellaneous      */
/***********************/
// Initial angle offset
float base_pos_offset = 0;
bool did_set_offset = false;

// dd localization
float pidError; // PID debugging for turret. See pid.h
float dummy1 = 0; // serves as dummy placeholder from original code.
float dummy2 = 0;

int packetCounter = 0;
long debug_time = 0;

/***************************/
/*      ROS                */
/***************************/
ros::NodeHandle nh;

std_msgs::Int32 loop_time_error_msg;
ros::Publisher pub_loop_error("/quori/base/loop_error", &loop_time_error_msg);

geometry_msgs::Vector3 v3_msg;
ros::Publisher pub_vel("/quori/base/vel_status", &v3_msg);

geometry_msgs::Vector3 desired_v3_msg;
ros::Publisher pub_desired_vel("/quori/base/desired_vel_status", &desired_v3_msg);
/*
Publishing another Vector3 is too much information over serial connection

geometry_msgs::Vector3 pid1_msg;
ros::Publisher pub_pid1("/quori/base/pid1", &pid1_msg);
*/
std_msgs::Float32 angle_msg;
ros::Publisher pub_pos("/quori/base/pos_status", &angle_msg);

/******************************/
/* ROS callback functions zzh */
/******************************/
void CallbackHolo (const geometry_msgs::Vector3& cmd_msg_holo) {
    last_command_time = micros();
    use_holonomic_drive = true;
    desired_h_xdot = cmd_msg_holo.x;
    desired_h_ydot = cmd_msg_holo.y;
    desired_h_rdot = cmd_msg_holo.z;
}

void CallbackDiff (const geometry_msgs::Vector3& cmd_msg_diff) {
    last_command_time = micros();
    use_holonomic_drive = false;
    desired_M1_v = cmd_msg_diff.x;
    desired_M2_v = cmd_msg_diff.y;
    desired_MT_v = cmd_msg_diff.z;
}

void CallbackOffset(const std_msgs::Float32& cmd_msg_offset) {
    last_command_time = micros();
    did_set_offset = true;
    base_pos_offset = cmd_msg_offset.data + base_pos_offset;
    if (base_pos_offset > 1) {
        base_pos_offset = base_pos_offset - 1;
    }
    if (base_pos_offset < 0) {
        base_pos_offset = base_pos_offset + 1;
  }
}

void CallbackM1 (const geometry_msgs::Vector3& msg) {
    speed_M1_pid.set_Kp(msg.x);
    speed_M1_pid.set_Ki(msg.y);
    speed_M1_pid.set_Kd(msg.z);
}

void CallbackM2 (const geometry_msgs::Vector3& msg) {
    speed_M2_pid.set_Kp(msg.x);
    speed_M2_pid.set_Ki(msg.y);
    speed_M2_pid.set_Kd(msg.z);
}

void CallbackMT (const geometry_msgs::Vector3& msg) {
    speed_MT_pid.set_Kp(msg.x);
    speed_MT_pid.set_Ki(msg.y);
    speed_MT_pid.set_Kd(msg.z);
}
  
//*********************//
//Setting Up Subscriber//
//*********************//
ros::Subscriber<geometry_msgs::Vector3> sub_holo("/quori/base/cmd_holo", CallbackHolo); //subscriber for holonomic mode
ros::Subscriber<geometry_msgs::Vector3> sub_diff("/quori/base/cmd_diff", CallbackDiff); //subscriber for diff mode
ros::Subscriber<std_msgs::Float32> sub_offset("/quori/base/cmd_offset", CallbackOffset); //subscriber for holonomic mode
ros::Subscriber<geometry_msgs::Vector3> sub_M1("/quori/base/set_M1Gains", CallbackM1); //subscriber for TODO. hide these after this is tuned so users do not accedentially break the robot
ros::Subscriber<geometry_msgs::Vector3> sub_M2("/quori/base/set_M2Gains", CallbackM2); //subscriber for 
ros::Subscriber<geometry_msgs::Vector3> sub_MT("/quori/base/set_MTGains", CallbackMT); //subscriber for MT

void init_angle_sensors() {
    angle_sensor_M1.frequency();
    angle_sensor_M2.frequency();// i made the clock 10Mhz. it was 1Mhz to star
    angle_sensor_MT.frequency();// i made the clock 10Mhz. it was 1Mhz to star
}

void init_pids() {
    speed_M1_pid.set_Kp(4);
    speed_M1_pid.set_Ki(0.5);
    speed_M1_pid.set_Kd(0);
    speed_M1_pid.set_saturation(1.0); // percent of motor voltage
    speed_M1_pid.set_feed_forward(0.0);

    speed_M2_pid.set_Kp(2);
    speed_M2_pid.set_Ki(8);
    speed_M2_pid.set_Kd(0);
    speed_M2_pid.set_saturation(1.0);// percent of motor voltage
    speed_M2_pid.set_feed_forward(0.0);

    speed_MT_pid.set_Kp(0.001);
    speed_MT_pid.set_Ki(0.005);
    speed_MT_pid.set_Kd(0);
    speed_MT_pid.set_saturation(1.0);// percent of motor voltage
    speed_MT_pid.set_feed_forward(0.0);
}

/**
   Debugging convenience method that blinks the builtin LED.
*/
bool toggle = 1;
DigitalOut myled(LED1);

void blink_times(int i) {
    for (int j = 0; j < i * 2; j++) {
        myled = toggle;
        delay(500);
        toggle = !toggle;
    }
}

float converted_time_elapsed() {
    return time_elapsed_micros * 0.000001;
}

void holonomic_drive() {
    h_xdot_cmd = desired_h_xdot;
    h_ydot_cmd = desired_h_ydot;
    h_rdot_cmd = desired_h_rdot;
    // using output of holonomic PID, compute jacobian values for motor inputs
    set_holonomic_desired_velocities(h_xdot_cmd, h_ydot_cmd, h_rdot_cmd); // set these setpoints to the output of the holonomic PID controllers
    //limit_holonomic_acceleration(MAX_HOLO_ACCEL);
    get_holonomic_motor_velocities(sensed_drive_angle * 2 * PI, &desired_M1_v, &desired_M2_v, &desired_MT_v);
}

void limit_motor_acceleration() {
    if (desired_M1_v - prev_desired_M1_v >= MAX_BASE_ACCEL) {
        desired_M1_v = prev_desired_M1_v + MAX_BASE_ACCEL;
    }
    else if (desired_M1_v - prev_desired_M1_v <= -MAX_BASE_ACCEL) {
        desired_M1_v = prev_desired_M1_v - MAX_BASE_ACCEL;
    }

    if (desired_M2_v - prev_desired_M2_v >= MAX_BASE_ACCEL) {
        desired_M2_v = prev_desired_M2_v + MAX_BASE_ACCEL;
    }
    else if (desired_M2_v - prev_desired_M2_v <= -MAX_BASE_ACCEL) {
        desired_M2_v = prev_desired_M2_v - MAX_BASE_ACCEL;
    }

    prev_desired_M1_v = desired_M1_v;
    prev_desired_M2_v = desired_M2_v;
}

void calculate_sensed_drive_angle() {
    const int* anglesT = angle_sensor_MT.read_angle();
    int angleT = anglesT[0];
    sensed_drive_angle = As5048Spi::mask(angleT) * TICKS2REV;
    if (!did_set_offset) {
        base_pos_offset = sensed_drive_angle;
        did_set_offset = true;
    }
    sensed_drive_angle = sensed_drive_angle - base_pos_offset;
    if (sensed_drive_angle < 0) {
        sensed_drive_angle = 1 + sensed_drive_angle;
    }
    if (sensed_drive_angle > 1) {
        sensed_drive_angle = -1 + sensed_drive_angle;
    }  
}

void set_speed_of_motors() { 
    if (!use_holonomic_drive) {
        limit_motor_acceleration();
    }

    speed_M1_pid.set_reference(desired_M1_v);

    if (desired_M1_v == 0) {
        // Deadband
        M1_v_cmd = 0;
        speed_M1_pid.Reset();
    }
    else if (desired_M1_v > 0) {
        float M1_feed_fwd = desired_M1_v * 1.171 + 0.12; 
        speed_M1_pid.set_feed_forward(M1_feed_fwd);
        M1_v_cmd =  speed_M1_pid.PidCompute(sensed_M1_v, time_elapsed_secs, time_elapsed_secs_inv);
    }
    else if (desired_M1_v<0) {
        float M1_feed_fwd = desired_M1_v * 1.171 - 0.12; 
        speed_M1_pid.set_feed_forward(M1_feed_fwd);
        M1_v_cmd =  speed_M1_pid.PidCompute(sensed_M1_v, time_elapsed_secs, time_elapsed_secs_inv);
    }

    set_speed(&M1_v_cmd, &serial_M1, right_motor_tag);

    speed_M2_pid.set_reference(desired_M2_v);

    if (desired_M2_v == 0) {
        // Deadband
        M2_v_cmd = 0;
        speed_M2_pid.Reset();
    }
    else if(desired_M2_v > 0) {
        float M2_feed_fwd = desired_M2_v * 1.171 + 0.12;
        speed_M2_pid.set_feed_forward(M2_feed_fwd);
        M2_v_cmd =  speed_M2_pid.PidCompute(sensed_M2_v, time_elapsed_secs, time_elapsed_secs_inv);
    }
    else if(desired_M2_v < 0) {
        float M2_feed_fwd = desired_M2_v * 1.171 - 0.12;
        speed_M2_pid.set_feed_forward(M2_feed_fwd);
        M2_v_cmd = speed_M2_pid.PidCompute(sensed_M2_v, time_elapsed_secs, time_elapsed_secs_inv);
    }

    set_speed(&M2_v_cmd, &serial_M2, left_motor_tag);
    speed_MT_pid.set_reference(desired_MT_v);

    if (desired_MT_v == 0) {
        // Deadband
        MT_v_cmd = 0;
        speed_MT_pid.Reset();
    }
    else if(desired_MT_v > 0) {
        float MT_feed_fwd = desired_MT_v * 0.004022 + 0.0175;
        speed_MT_pid.set_feed_forward(MT_feed_fwd);
        MT_v_cmd = speed_MT_pid.PidCompute(sensed_MT_v, time_elapsed_secs, time_elapsed_secs_inv);
    }
    else if(desired_MT_v < 0) {
        float MT_feed_fwd = desired_MT_v * 0.004022 - 0.0175;
        speed_MT_pid.set_feed_forward(MT_feed_fwd);
        MT_v_cmd = speed_MT_pid.PidCompute(sensed_MT_v, time_elapsed_secs, time_elapsed_secs_inv);
    }

    set_speed_of_turret(&MT_v_cmd, &serial_MT);
}

/**********************/
/* Messaging Protocol */
/**********************/
/**
   Uses ROS to send update
*/
void ros_pub_loop() {
    loop_time_error_msg.data = debug_time;//loop_time_error for only errors
    loop_time_duration = 0;
    pub_loop_error.publish(&loop_time_error_msg);
}

void publish_sensed_drive_angle() {
    angle_msg.data = sensed_drive_angle;
    pub_pos.publish(&angle_msg);
}

void publish_pid_val() {
    //pid1_msg.x = speed_M1_pid.get_Kp();
    //pid1_msg.y = speed_M1_pid.get_Ki();
    //pid1_msg.z = speed_M1_pid.get_Kd();
    //pub_pid1.publish(&pid1_msg);
}

void ros_telemetry() {
    v3_msg.x = sensed_M1_v;
    v3_msg.y = sensed_M2_v;
    v3_msg.z = sensed_MT_v;
    pub_vel.publish(&v3_msg);

    publish_sensed_drive_angle();

    desired_v3_msg.x = desired_M1_v;
    desired_v3_msg.y = desired_M2_v;
    desired_v3_msg.z = desired_MT_v;
    pub_desired_vel.publish(&desired_v3_msg);
}

/**
   Message handler that does necessary actions with the information provided.
   @param msg_manager Instance of the message manager to identify incoming messages
   @param val The message packet that has yet to be identified.
*/
void handle_message(MESSAGE_MANAGER_t* msg_manager, char* val) {
    uint8_t msg_id = val[0];
    switch (msg_id) {
    case HolonomicVelocityMessageType: {
        use_holonomic_drive = true;
        HolonomicVelocity *msg = (HolonomicVelocity*) val;
        msg_manager->holo_vel_struct = *msg;
        desired_h_xdot = msg_manager->holo_vel_struct.x_dot;
        desired_h_ydot = msg_manager->holo_vel_struct.y_dot;
        desired_h_rdot = msg_manager->holo_vel_struct.r_dot;
        break;
    }

    case DifDriveVelocityMessageType: {
        DifDriveVelocity *msg = (DifDriveVelocity*) val;
        msg_manager->dif_drive_vel_struct = *msg;
        use_holonomic_drive = false;
        desired_M1_v = msg_manager->dif_drive_vel_struct.left_v;
        desired_M2_v = msg_manager->dif_drive_vel_struct.right_v;
        desired_MT_v = msg_manager->dif_drive_vel_struct.turret_v;
        break;
    }

    // Tests on command
    case -100:
        // Square Test
        square_test_did_start = true;
        break;

    case -101:
        // Right Angle Test
        right_test_did_start = true;
        break;

    case -102:
      //Circle Test
        circle_test_did_start = true;
        break;

    case -103:
        //Spiral Test
        spiral_test_did_start = true;
        break;

    case -104:
        //Sinusoid Test
        sine_test_did_start = true;
        break;

    case -105:
        //Heading Circle
        heading_circle_test_did_start = true;
        break;

    default:
        printf("Message not recognized.");
        break;
    }
}

/******************************************************/
/*    Encoder reading and Velocity Calculations   */
/******************************************************/

/**
   Calculates the difference of ticks between previously sensed and currently sensed readings. Also considers the
   case when there is an overflow between readings (ie, encoder reads 4090, moves forward, then reads 2 at time t+1)

   @param prev Previously sensed value
   @param current Currently sensed value
   @param max The encoder's max reading
   @param lim_min Desired floor at which you determine there has been an overflow
   @param lim_max Desired ceiling at which you determine that there has been an overflow.
   @returns The absolute change between the prev and current values.
*/

float calculate_decoder_count_change(int prev, int current, int res_max) {
    int diff = current - prev;
    int res_min = 0;
    if (abs(diff) > res_max / 2) {
        if (current >= prev) {
            return ((-1) * ((res_max - current) + (prev - res_min) + 1));
        }
        else {
            return ((res_max - prev) + (current - res_min) + 1);
        }
    }
    else {
        return diff;
    }
}

/**
   Computes the instantaneous velocity from the previous and current encoder reading, then passes it through a filter.
*/
void compute_sensed_motor_velocities() {
    // reading all encoders and clearing erros takes about 360 us. if the as5048A.cpp file is changed so the rate of transmission is settings = SPISettings(10000000, MSBFIRST, SPI_MODE1); and not 1MHz
    // there is a periodic spike in velocity when the position rolls over of about +/-0.05m/s
    const int* angles = angle_sensor_M1.read_angle();
    int angle = angles[0];
    decoder_count_M1 = As5048Spi::mask(angle);

    const int* angles2 = angle_sensor_M2.read_angle();
    int angle2 = angles2[0];
    decoder_count_M2 = As5048Spi::mask(angle2);

    const int* anglesT = angle_sensor_MT.read_angle();
    int angleT = anglesT[0];
    decoder_count_MT = TICKS_PER_REV_DDRIVE - As5048Spi::mask(angleT);

    // Clear the angle sensor errors.
    angle_sensor_M1.error();
    angle_sensor_M2.error();
    angle_sensor_MT.error();

    // Calculating the difference between prev and current sensed positions
    float decoder_count_change_M1 = calculate_decoder_count_change(decoder_count_M1_prev, decoder_count_M1, 16383);
    float decoder_count_change_M2 = calculate_decoder_count_change(decoder_count_M2_prev, decoder_count_M2, 16383);
    float decoder_count_change_MT = calculate_decoder_count_change(decoder_count_MT_prev, decoder_count_MT, 16383);

    samples_M1.add((int)decoder_count_change_M1);
    samples_M2.add((int)decoder_count_change_M2);
    samples_MT.add((int)decoder_count_change_MT);
    decoder_count_change_M1 = (float)samples_M1.getMedian();
    decoder_count_change_M2 = (float)samples_M2.getMedian();
    decoder_count_change_MT = (float)samples_MT.getMedian();

    // Setting the previous encoder values
    decoder_count_M1_prev = decoder_count_M1;
    decoder_count_M2_prev = decoder_count_M2;
    decoder_count_MT_prev = decoder_count_MT;

    float current_vel_right = get_speed_from_difference(decoder_count_change_M1, time_elapsed_secs_inv); // m/s
    float current_vel_left = get_speed_from_difference(decoder_count_change_M2, time_elapsed_secs_inv); // m/s
    float current_vel_turret = get_ang_speed_from_difference(decoder_count_change_MT, time_elapsed_secs_inv);// degs/s

    sensed_M1_v = m1_filter_speed.sample(current_vel_right);
    sensed_M2_v = m2_filter_speed.sample(current_vel_left);
    sensed_MT_v = mt_filter_speed.sample(current_vel_turret);
}

float low_pass_velocity_filter(float current, float prev) {
    // Simple low pass filter
    float beta = 0.386; // Calculated 0.386, but 0.6 works well
    return beta * current + (1 - beta) * prev; //filter at 10 hz - Tarik.
}

int main() {
    nh.getHardware()->setBaud(115200);
    StartArduinoTimer();
    init_pids();
    delay(1000);
    start_time = millis();
    init_angle_sensors();
    serial_setup();

    nh.initNode();
    nh.advertise(pub_vel);
    nh.advertise(pub_desired_vel);
    nh.advertise(pub_loop_error);
    nh.advertise(pub_pos);
    //nh.advertise(pub_pid1);

    nh.subscribe(sub_holo); 
    nh.subscribe(sub_diff);
    nh.subscribe(sub_offset);
    nh.subscribe(sub_M1);
    nh.subscribe(sub_M2);
    nh.subscribe(sub_MT);

    delay(1);
    while (1) {
        loop_time_duration = micros() - last_recorded_time;
        if (loop_time_duration >= LOOPTIME) {
            time_elapsed_secs = ((float)loop_time_duration) * 0.000001;
            time_elapsed_secs_inv = 1.0/time_elapsed_secs;// if loop is stable enough and readings are not too sensitive then comment this out and replace with contants DT and DT_INV

            last_recorded_time = micros();
            if ((micros() - last_update_msg_time) > UPDATE_MSG_TIME) {
                last_update_msg_time = micros();// should be before the res_telemetry code
                ros_telemetry();//zzh sends sensed M1 M2 MT data to "vel_status" topic in ROS. for some reason ros_telemetry in here runs at about half the speed.  
                ros_pub_loop();
                publish_pid_val();
                nh.spinOnce();//zzh node spins once and subscribing to diff & holo input from ROS
            }
            debug_time = time_elapsed_micros;
            
            unsigned long time_diff = micros() - last_command_time;
            if (time_diff > COMMAND_TIMEOUT) {
                desired_h_xdot = 0;
                desired_h_ydot = 0;
                desired_h_rdot = 0;
                desired_M1_v = 0;
                desired_M2_v = 0;
                desired_MT_v = 0;
            }

            compute_sensed_motor_velocities();//750 us 
            calculate_sensed_drive_angle();//70us

            if (use_holonomic_drive) {
                holonomic_drive();//430us
            }

            set_speed_of_motors();
        }

        if (loop_time_duration > LOOPTIME) { // catch exceptionally long delays
            loop_time_error = loop_time_duration;
        }
    }
}