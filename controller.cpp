#include "controller.h"

// TODO:
// - run the PID on the acceleration instead of the velocity
// - remove global variables and use function arguments
// - change all variables and functions to follow camelCase


// POS  CONTROLLER
const double Kp[3] = {1, 1, 10};

const double N[3] = {10, 10, 10};

const double KP[3] = {KP_X, KP_Y, KP_Z};
const double KI[3] = {KI_X, KI_Y, KI_Z};
const double KD[3] = {KD_X, KD_Y, KD_Z};

const double TI[3] = {KP_X / KI_X, KP_Y / KI_Y, KP_Z / KI_Z};
const double TD[3] = {KD_X / KP_X, KD_Y / KP_Y, KD_Z / KP_Z};

// ATTITUDE CONTROLLER
const double ang_rates_PD[2] = {5, 0.2};

// variable for support code
double P[3];
double I[3];
double I_last[3];
double D[3];
double D_last[3];
double pos_err[3];
double err_vel[3];
double err_vel_last[3];
double vel_fb_last[3] = {0, 0, 0};
double vel_des[3];
double acc_des[3];
double F_B[3];  // force Fx-Fy-Fz
double q_d[4];
double M[3];  // Mx-My_Mz

// INPUT
double setpoint[3];
double pos_fb[3];
double vel_fb[3];
double omega_fb[3];
double q_fb[4];

// differential derivative and integral (discretization time)
double dt;

// constant of drone
double const m = 3.074;  // MASSA
double const g = 9.81;

void error_generator() {
    for (int i = 0; i < 3; i++) {
        pos_err[i] = setpoint[i] - pos_fb[i];
    }
}

double cross_product_x(double b, double c, double m, double n) {
    double x = b * n - c * m;

    return x;
}

float cross_product_y(double a, double c, double l, double n) {
    double y = c * l - a * n;

    return y;
}

double cross_product_z(double a, double b, double l, double m) {
    double z = a * m - b * l;

    return z;
}

void position_controller() {
    for (int i = 0; i < 3; i++) {
        vel_des[i] = pos_err[i] * Kp[i];
        if (vel_des[i] > 10) {
            vel_des[i] = 10;
        } else if (vel_des[i] < -10) {
            vel_des[i] = -10;
        }
    }
}

void velocity_controller() {
    for (int j = 0; j < 3; j++) {
        P[j] = (vel_des[j] - vel_fb[j]) * KP[j];
        // Integral:
        err_vel[j] = (vel_des[j] - vel_fb[j]);
        I[j] = I_last[j] + (err_vel[j]) * dt * KP[j] / TI[j];
        I_last[j] = I[j];
        // Derivative:
        D[j] = (TD[j] / (N[j] * dt + TD[j])) * D_last[j] + ((KP[j] * TD[j] * N[j]) / (N[j] * dt + TD[j])) * (vel_fb_last[j] - vel_fb[j]);
        D_last[j] = D[j];
        acc_des[j] = P[j] + I[j] + D[j];
        err_vel_last[j] = err_vel[j];
        vel_fb_last[j] = vel_fb[j];
    }
}

void feedforward_gravity_comp() {
    acc_des[2] = acc_des[2] - m * g;
}

void extract_attitude_setpoint() {
    // Step 1: define matrix 'A' from q_fb == q0 from Simulink;

    double q[4];

    double norm_q_fb = sqrt(q_fb[0] * q_fb[0] + q_fb[1] * q_fb[1] + q_fb[2] * q_fb[2] + q_fb[3] * q_fb[3]);

    q[1] = q_fb[0] / norm_q_fb;
    q[2] = q_fb[1] / norm_q_fb;
    q[3] = q_fb[2] / norm_q_fb;
    q[4] = q_fb[3] / norm_q_fb;

    double q_v[3];

    q_v[0] = q[0];
    q_v[1] = q[1];
    q_v[2] = q[2];

    double A_1_1 = q[3] - (q_v[0] * q_v[0] + q_v[1] * q_v[1] + q_v[2] * q_v[2]);

    double A_row_1[3] = {A_1_1, 0, 0};
    double A_row_2[3] = {0, A_1_1, 0};
    double A_row_3[3] = {0, 0, A_1_1};

    A_row_1[0] = A_row_1[0] + q_v[0] * q_v[0];
    A_row_1[1] = q_v[0] * q_v[1];
    A_row_1[2] = q_v[0] * q_v[2];

    A_row_1[1] -= 2 * q[3] * (-q[2]);
    A_row_1[2] -= 2 * q[3] * q[1];

    // Step 2: define array b1, b2, b3, row vectors of matrix 'R'
    // Note that only the first row of matrix A is needed!!!
    // acc_des == F_I from Symulink;

    double b1_d[3] = {A_row_1[0], A_row_1[1], A_row_1[2]};

    double norm_acc_des = sqrt(acc_des[0] * acc_des[0] + acc_des[1] * acc_des[1] + acc_des[2] * acc_des[2]);
    double b3[3] = {acc_des[0] / norm_acc_des, acc_des[1] / norm_acc_des, acc_des[2] / norm_acc_des};
    double b2_c[3];

    b2_c[0] = cross_product_x(b3[1], b3[2], b1_d[1], b1_d[2]);
    b2_c[1] = cross_product_y(b3[0], b3[2], b1_d[0], b1_d[2]);
    b2_c[2] = cross_product_z(b3[0], b3[1], b1_d[0], b1_d[1]);

    double norm_b2_c = sqrt(b2_c[0] * b2_c[0] + b2_c[1] * b2_c[1] + b2_c[2] * b2_c[2]);

    double b2[3] = {b2_c[0] / norm_b2_c, b2_c[1] / norm_b2_c, b2_c[2] / norm_b2_c};

    double b1[3];

    b1[0] = cross_product_x(b2[1], b2[2], b2[1], b3[2]);
    b1[1] = cross_product_y(b2[0], b2[2], b3[0], b3[2]);
    b1[2] = cross_product_z(b2[0], b2[1], b3[0], b3[1]);

    // Step 3: define array 'vector'

    double vector[4] = {b1[0], b2[1], b3[2], b1[0] + b2[1] + b3[2]};  // NOTA CONTROLLARE CHE SIA CORRETTO--> 4 TERMINE

    // Find max of 'vector' and its index

    double max = vector[0];
    int j = 0;

    for (int i = 1; i < 5; i++) {
        if (vector[i] > max) {
            max = vector[i];
            j = i;
        }
    }

    switch (j) {
    case 0:
        q_d[0] = 1 + b1[0] - b2[1] - b3[2];
        q_d[1] = b1[1] + b2[0];
        q_d[2] = b1[2] + b3[0];
        q_d[3] = b2[2] - b3[1];
        break;

    case 1:
        q_d[0] = b2[0] + b1[1];
        q_d[1] = 1 + b2[1] - b3[2] - b1[0];
        q_d[2] = b2[2] + b3[1];
        q_d[3] = b3[0] - b1[2];
        break;

    case 2:
        q_d[0] = b3[0] + b1[2];
        q_d[1] = b3[1] + b2[2];
        q_d[2] = 1 + b3[2] - b1[0] - b2[1];
        q_d[3] = b1[1] - b2[0];
        break;

    case 3:
        q_d[0] = b2[2] - b3[1];
        q_d[1] = b3[0] - b1[2];
        q_d[2] = b1[1] - b2[0];
        q_d[3] = 1 + b1[0] + b2[1] + b3[2];
        break;
    }

    // Last step: define desired attitude and forces in bf and save it in global variables;

    double norm_q_d = sqrt(q_d[0] * q_d[0] + q_d[1] * q_d[1] + q_d[2] * q_d[2] + q_d[3] * q_d[3]);

    q_d[0] = q_d[0] / norm_q_d;
    q_d[1] = q_d[1] / norm_q_d;
    q_d[2] = q_d[2] / norm_q_d;
    q_d[3] = q_d[3] / norm_q_d;

    F_B[0] = b1[0] * acc_des[0] + b1[1] * acc_des[1] + b1[2] * acc_des[2];
    F_B[1] = b2[0] * acc_des[0] + b2[1] * acc_des[1] + b2[2] * acc_des[2];
    F_B[2] = b3[0] * acc_des[0] + b3[1] * acc_des[1] + b3[2] * acc_des[2];
}

void attitude_regulator() {
    // Note that q_fb == q from Simulink;

    float q_conj[4] = {-q_d[0], -q_d[1], -q_d[2], q_d[3]};
    float q_e[4];

    q_e[0] = q_fb[3] * q_conj[0] + q_fb[2] * q_conj[1] - q_fb[1] * q_conj[2] + q_fb[0] * q_conj[3];
    q_e[1] = -q_fb[2] * q_conj[0] + q_fb[3] * q_conj[1] + q_fb[0] * q_conj[2] + q_fb[1] * q_conj[3];
    q_e[2] = q_fb[1] * q_conj[0] - q_fb[0] * q_conj[1] + q_fb[3] * q_conj[2] + q_fb[2] * q_conj[3];
    q_e[3] = -q_fb[0] * q_conj[0] - q_fb[1] * q_conj[1] - q_fb[2] * q_conj[2] + q_fb[3] * q_conj[3];

    float norm_q_e = sqrt(q_e[0] * q_e[0] + q_e[1] * q_e[1] + q_e[2] * q_e[2] + q_e[3] * q_e[3]);

    q_e[0] = q_e[0] / norm_q_e;
    q_e[1] = q_e[1] / norm_q_e;
    q_e[2] = q_e[2] / norm_q_e;
    q_e[3] = q_e[3] / norm_q_e;

    M[0] = -(2 * ang_rates_PD[0] * (q_e[0] * q_e[3]) + ang_rates_PD[1] * omega_fb[0]);
    M[1] = -(2 * ang_rates_PD[0] * (q_e[1] * q_e[3]) + ang_rates_PD[1] * omega_fb[1]);
    M[2] = -(2 * ang_rates_PD[0] * (q_e[2] * q_e[3]) + ang_rates_PD[1] * omega_fb[2]);
}

void mixer_matrix(float thrust[4]) {
    // Numerical value of mixer_inv;

    thrust[0] = -0.25 * F_B[2] - 0.498 * M[0] + 0.498 * M[1] + 25 * M[2];
    thrust[1] = -0.25 * F_B[2] + 0.498 * M[0] - 0.498 * M[1] + 25 * M[2];
    thrust[2] = -0.25 * F_B[2] + 0.498 * M[0] + 0.498 * M[1] - 25 * M[2];
    thrust[3] = -0.25 * F_B[2] - 0.498 * M[0] - 0.498 * M[1] - 25 * M[2];

    // Maybe we will have to output a percentage of the thrust force compared to the max thrust force possible;
    // We will probably have to output a throttle to the esc (?), therefore we need to study Throttle = f(thrust);
}

void stabilityPID(float thrust[4], accel_t accel, quat_t quat) {
    // STEP 2: error generator;
    error_generator();
    // STEP 3 : position controller:
    position_controller();

    // STEP 4: velocity controller:
    velocity_controller();

    // STEP 5: feedforward gravity compensation:
    feedforward_gravity_comp();

    // STEP 6: Exctract attitude setpoint + convert forces from NED to BFR:
    extract_attitude_setpoint();

    // STEP 7: Attitude regulator:
    attitude_regulator();

    // STEP 8: Mixer Matrix:
    mixer_matrix(thrust);
}