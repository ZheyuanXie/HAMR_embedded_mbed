#include "holonomic_control.h"
#include "math.h"
#include "constants.h"

#define DIM_A WHEEL_DIST//0.1352065
#define DIM_B WHEEL_DIST//0.1352065
#define DIM_B_INV 1/WHEEL_DIST
#define DIM_R WHEEL_RADIUS

// this value will determine how quickly to change the setpoint
float max_linear_acceleration = .1;
float max_angular_acceleration = .1;

// desired user velocities
float setpoint_x = 0;
float setpoint_y = 0;
float setpoint_r = 0;

/* last known state (updated by update_holonomic_state()) */
float state_xdot = 0;
float state_ydot = 0;
float sensed_theta_d = 0; //theta_d is the angle of the drive relative to turret


// Actual velocities input into Jacobian. 
// These depend on the set_point and max_linear_acceleration
float desired_x = 0;
float desired_y = 0;
float desired_r = 0;

//last set of velocities output from jacobian 
float output_m1 = 0;
float output_m2 = 0;
float output_mt = 0;

float prev_setpoint_x = 0;
float prev_setpoint_y = 0;
float prev_setpoint_r = 0;

//debugging varibles
unsigned long last_time_debug_loop = 0;

 
// Update holonomic actuator velocities with given current known state. 
// Return the computed output in the output pointers
void get_holonomic_motor_velocities(float _state_theta_dheta_d, 
                                    float* _output_m1, 
                                    float* _output_m2,
                                    float* _output_mt) {
    sensed_theta_d = _state_theta_dheta_d;

    desired_x = setpoint_x;
    desired_y = setpoint_y;
    desired_r = setpoint_r;

    compute_ramsis_jacobian(desired_x, desired_y, desired_r, sensed_theta_d);

    * _output_m1 = -1 * output_m1;
    * _output_m2 = output_m2;
    * _output_mt = output_mt;
}

void compute_global_state(float sensed_m1, 
                        float sensed_m2,
                        float sensed_mt,
                        float sensed_t,
                        float* xdot,
                        float* ydot,
                        float* tdot){
    // Computes the current x, y, and t dot from given information
    float sint, cost;
    float b_s, b_c, a_s, a_c;
    float rac, rbc, ras, rbs;

    float converted_m1 = sensed_m1/WHEEL_RADIUS;
    float converted_m2 = sensed_m2/WHEEL_RADIUS;
    float converted_mt = sensed_mt * (PI/180);
 
    sint = sin(sensed_t);
    cost = cos(sensed_t);

    b_s = DIM_B * sint; 
    b_c = DIM_B * cost;
    a_s = DIM_A * sint;
    a_c = DIM_A * cost;

    rac = DIM_R * a_c;
    rbc = DIM_R * b_c;
    ras = DIM_R * a_s;
    rbs = DIM_R * b_s;

    // these pointers are: computed_xdot, computed_ydot and computed_tdot
    *xdot = ((-rbc - ras)*converted_m1 + (rbc - ras) * converted_m2) / (2.0 * DIM_A);
    *ydot = ((-rbs + rac)*converted_m1 + (rbs + rac) * converted_m2) / (2.0 * DIM_A);
    *tdot = (DIM_R * converted_m1 - DIM_R * converted_m2 )/ (2.0 * DIM_A) - converted_mt;
}


// Helper Function for update_holonomic_state()
void compute_ramsis_jacobian(float desired_xdot,
                            float desired_ydot,
                            float desired_tdot,
                            float t){
    // Computed in radians....
    float sint, cost;
    float b_sin, b_cos, a_sin, a_cos;
    sint = sin(t);//125us
    cost = cos(t);//125us
   
    b_sin = DIM_B * sint;//7us
    b_cos = DIM_B * cost;
    a_sin = DIM_A * sint;
    a_cos = DIM_A * cost;
    
    // jacobian takes about 60us (0 as inputs) to 120us to complete all three
    output_m1 = (desired_xdot*(-b_sin - a_cos) + desired_ydot*(b_cos - a_sin))*(DIM_B_INV);
    output_m2 = (desired_xdot*(-b_sin + a_cos) + desired_ydot*(b_cos + a_sin))*(DIM_B_INV);
    output_mt   = (RAD2DEG) * ((-desired_xdot * cost - desired_ydot * sint) *DIM_B_INV - desired_tdot);// notice this is the negative of the listed jacobian. this is because the motor velocity is the negative of the turret velocity.

    // Scale the velocities linearly if a maximuim was reached
    float scale = 1;
    float temp = 0;
    if (abs(output_mt) > MAX_MT_VEL) {
      scale = MAX_MT_VEL / abs(output_mt);
    } 
    if (abs(output_m1) > MAX_M1_VEL) {
      temp = MAX_M1_VEL / abs(output_m1);
      if (scale > temp){
      scale = temp;
      }
    }
    if (abs(output_m2) > MAX_M2_VEL) {
      temp = MAX_M2_VEL / abs(output_m2);
      if (scale > temp){
      scale = temp;
      }
    }
    // scale velocities to be within the achieveable region while preserving the unit direction. mt is degrees per second. m1 and m2 are m/s 0.6~80rpm
    output_m1 = scale*output_m1;
    output_m2 = scale*output_m2;
    output_mt = scale*output_mt;
}

void set_holonomic_desired_velocities(float xdot, float ydot, float rdot) {
    // set in-file variables to commanded x, y, and r
    setpoint_x = xdot;
    setpoint_y = ydot;
    setpoint_r = rdot;
}

void set_max_linear_acceleration(float a){
    max_linear_acceleration = a;
}

void set_max_angular_acceleration(float a){
    max_angular_acceleration = a;
}

void limit_holonomic_acceleration(float limit) {
    float scale = 1;
    float curr_vel = sqrtf(pow(setpoint_x, 2) + pow(setpoint_y, 2));
    float prev_vel = sqrtf(pow(prev_setpoint_x, 2) + pow(prev_setpoint_y, 2));
    if (curr_vel - prev_vel > limit) {
        scale = (limit - prev_vel) / curr_vel;
        setpoint_x = setpoint_x * scale;
        setpoint_y = setpoint_y * scale;
        setpoint_r = setpoint_r * scale;
    }
    else if (curr_vel - prev_vel < -limit) {
        scale = (-limit + prev_vel) / prev_vel;
        setpoint_x = prev_setpoint_x * scale;
        setpoint_y = prev_setpoint_y * scale;
        setpoint_r = prev_setpoint_r * scale;
    }
    prev_setpoint_x = setpoint_x;
    prev_setpoint_y = setpoint_y;
    prev_setpoint_r = setpoint_r;
}



