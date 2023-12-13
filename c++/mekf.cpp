#include "mekf.h"
#include "mekf_math_operations.h"
#include<vector>
#include<cmath>
#include<iostream>

/* State Vector 
 * gyro - accel - mag sensors
 * [0-2] error in quaternion parameter
 * [3-5] error in velocity 
 * [6-8] error in position 
 * [9-11] delta gyro bias 
 * [12-14] delta accel bias
 * [15-17] delta mag bias 
*/

// initializer
mekf::mekf(){
    // fixed dont change
    vec_size = 3;
    quat_size = 4;
    state_size = 18;
    num_obsv = 2;

    // can be edited //
    /* Should fill in all the variances
     * 
     * system model
     * model = truth + bias + noise
     * here bias - 1D random walk
     * bias_k = bias_k-1 + nosie 
     * 
     * observation model
     * obsv = truth + noise
     */
    gyro_var = 0.05; 
    accel_var = 0.05; 
    mag_var = 0.05; 

    gyro_bias_var = 0.001;
    accel_bias_var = 0.001;
    mag_bias_var = 0.001; 

    accel_obsv_var = 0.1;
    mag_obsv_var = 0.1;

    // initialize mekf vars
    mekf::init_vars();

}


/*
 * Initilaize all imp vars
 */
void mekf::init_vars(){

    // initialization
    std::vector<float> state_est(state_size);
    quat_est = {1, 0, 0, 0};
    cov_est = mat_num_mul(10, mat_eye(state_size));
    gyro_bias = {0, 0, 0};
    accel_bias = {0, 0, 0};
    mag_bias = {0, 0, 0};

    // build observer covariance matrix
    std::vector<std::vector<float>> obsv_cov_build(num_obsv*vec_size, std::vector<float>(num_obsv*vec_size));
    mat_assign(obsv_cov_build, mat_num_mul(accel_obsv_var, mat_eye(vec_size)), 0,2, 0,2); 
    mat_assign(obsv_cov_build, mat_num_mul(mag_obsv_var, mat_eye(vec_size)), 3,5, 3,5);
    obsv_cov = obsv_cov_build;
}

/*
 * Set reference vectors
*/
void mekf::set_accel_ref(std::vector<float> ref_vec) {
    // set reference acceleration vector
    accel_ref = ref_vec;
}

void mekf::set_mag_ref(std::vector<float> ref_vec){

    // set reference magnetic field vector
    mag_ref = ref_vec;
}

/* 
 * Get important quantities 
 */
std::vector<float> mekf::get_eul_est(){

    // get euler angles
    std::vector<float> eul_est(vec_size);
    eul_est = quat_to_eul(quat_est);
    return eul_est;
}

std::vector<float> mekf::get_quat_est(){ 
    
    // euler estimate
    return quat_est; 
}

std::vector<std::vector<float>> mekf::get_cov_est(){
    
    // covariance estimate
    return cov_est;
}

std::vector<float> mekf::get_diag_cov_est(){
    
    // diag of covariance estimate
    std::vector<float> diag_cov_est(state_size);

    for (int ii = 0; ii < state_size; ii++){
        diag_cov_est[ii] = cov_est[ii][ii];
    }

    return diag_cov_est;
}

std::vector<float> mekf::get_gvec_est(){

    // get gravity vector in the body frame
    return mat_vec_mul(quat_to_dcm(quat_est), accel_ref);
}



/*
 * One Step of MEKF 
*/
void mekf::mekf_step(float delta_t, std::vector<float> gyro_meas, std::vector<float> accel_meas, std::vector<float> mag_meas){

    // subtract bias from meas
    gyro_meas = vec_sub(gyro_meas, gyro_bias);
    accel_meas = vec_sub(accel_meas, accel_bias);
    mag_meas = vec_sub(mag_meas, mag_bias);

    //// Propagation Steps
    std::vector<float> omega_quat = {0, gyro_meas[0], gyro_meas[1], gyro_meas[2]};
    quat_est = vec_add(quat_est, vec_num_mul(delta_t/2, omega_quat));
    quat_est = quat_unit(quat_est);

    // model update
    std::vector<std::vector<float>> Phi(state_size, std::vector<float>(state_size));
    Phi = mekf::get_Phi(delta_t, gyro_meas, accel_meas);

    // process matrix
    std::vector<std::vector<float>> Q(state_size, std::vector<float>(state_size));
    Q = mekf::get_Q(delta_t);

    // prior covariance update 
    cov_est = mat_add( mat_mul(Phi, mat_mul(cov_est, mat_T(Phi))),  Q);

    //// Update Steps
    std::vector<std::vector<float>> H(num_obsv*vec_size, std::vector<float>(state_size));
    H = mekf::get_H();

    // compute the Kalman gain
    std::vector<std::vector<float>> K(state_size, std::vector<float>(num_obsv*vec_size));
    std::vector<std::vector<float>> inv_mat_term(num_obsv*vec_size, std::vector<float>(num_obsv*vec_size));
 
    inv_mat_term = mat_add(mat_mul(H, mat_mul(cov_est, mat_T(H))), obsv_cov);
    K = mat_mul(cov_est, mat_mul(mat_T(H), cholsl(inv_mat_term)));

    // error calculation
    std::vector<float> accel_err(vec_size);
    std::vector<float> mag_err(vec_size);
    std::vector<float> err(num_obsv*vec_size);
    
    accel_err = vec_sub(accel_meas, mat_vec_mul(quat_to_dcm(quat_est), accel_ref) );
    mag_err = vec_sub(mag_meas, mat_vec_mul(quat_to_dcm(quat_est), mag_ref) );
    err = vec_join(accel_err, mag_err);

    // posterior state 
    state_est = mat_vec_mul(K, err);

    // posterior cov step
    cov_est = mat_mul(mat_sub(mat_eye(state_size), mat_mul(K,H)) , cov_est);

    // unpack delta biases
    std::vector<float> delta_gyro_bias = {state_est[9], state_est[10], state_est[11]};
    std::vector<float> delta_accel_bias = {state_est[12], state_est[13], state_est[14]};
    std::vector<float> delta_mag_bias = {state_est[15], state_est[16], state_est[17]};

    // update bias terms
    gyro_bias = vec_add(gyro_bias, delta_gyro_bias);
    accel_bias = vec_add(accel_bias, delta_accel_bias);
    mag_bias = vec_add(mag_bias, delta_mag_bias);

    // posterior quat est
    std::vector<float> param_quat = {1, state_est[0]/2, state_est[1]/2, state_est[2]/2};
    quat_est = quat_mul(quat_est, param_quat);
    quat_est = quat_unit(quat_est);

    // check for nan and re-initialize
    // checking for only one element but works
    if (isnan(quat_est[0])){
        mekf::init_vars();
    }

}


/* 
 * Funtion to get Q (Process Covariance Matrix)
 */
std::vector<std::vector<float>> mekf::get_Q(float delta_t){

    std::vector<std::vector<float>> Q(state_size, std::vector<float>(state_size)); 

    // define the noise covarinace matrices
    std::vector<std::vector<float>> gyro_cov, accel_cov;
    gyro_cov = mat_num_mul(gyro_var, mat_eye(vec_size));
    accel_cov = mat_num_mul(accel_var, mat_eye(vec_size));
    // mag_cov = mat_num_mul(mag_var, mat_eye(vec_size));

    // define bias covariance matrices
    std::vector<std::vector<float>> gyro_bias_cov, accel_bias_cov, mag_bias_cov;
    gyro_bias_cov = mat_num_mul(gyro_bias_var, mat_eye(vec_size));
    accel_bias_cov = mat_num_mul(accel_bias_var, mat_eye(vec_size));
    mag_bias_cov = mat_num_mul(mag_bias_var, mat_eye(vec_size));

    // start building Q
    mat_assign(Q, mat_add( mat_num_mul(delta_t, gyro_cov), mat_num_mul(pow(delta_t,3)/3, gyro_bias_cov)), 0,2, 0,2);
    mat_assign(Q, mat_num_mul(-pow(delta_t,2)/2, gyro_bias_cov), 0,2, 9,11);

    mat_assign(Q, mat_add( mat_num_mul(delta_t, accel_cov), mat_num_mul(pow(delta_t,3)/3, accel_bias_cov)), 3,5, 3,5);
    mat_assign(Q, mat_add( mat_num_mul(pow(delta_t,2)/2, accel_cov), mat_num_mul(pow(delta_t,4)/8, accel_bias_cov)), 3,5, 6,8);
    mat_assign(Q, mat_num_mul(-pow(delta_t,2)/2, accel_bias_cov), 3,5, 12,14);

    mat_assign(Q, mat_add( mat_num_mul(pow(delta_t,2)/2, accel_cov), mat_num_mul(pow(delta_t,4)/8, accel_bias_cov)), 6,8, 3,5);
    mat_assign(Q, mat_add( mat_num_mul(pow(delta_t,3)/3, accel_cov), mat_num_mul(pow(delta_t,5)/20, accel_bias_cov)), 6,8, 6,8);
    mat_assign(Q, mat_num_mul(-pow(delta_t,3)/6, accel_bias_cov), 6,8, 12,14);

    mat_assign(Q, mat_num_mul(-pow(delta_t,2)/2, gyro_bias_cov), 9,11, 0,2);
    mat_assign(Q, mat_num_mul(delta_t, gyro_bias_cov), 9,11, 9,11);

    mat_assign(Q, mat_num_mul(-pow(delta_t,2)/2, accel_bias_cov), 12,14, 3,5);
    mat_assign(Q, mat_num_mul(-pow(delta_t,3)/6, accel_bias_cov), 12,14, 6,8);
    mat_assign(Q, mat_num_mul(delta_t, accel_bias_cov), 12,14, 12,14);

    mat_assign(Q, mat_num_mul(delta_t, mag_bias_cov), 15,17, 15,17);

    return Q; 

}


/* 
 * Function to get H 
 */
std::vector<std::vector<float>> mekf::get_H(){

    std::vector<std::vector<float>> H(num_obsv*vec_size, std::vector<float>(state_size)); 

    // start building H 
    mat_assign(H, vec_to_skewmat(mat_vec_mul(quat_to_dcm(quat_est), accel_ref)), 0,2, 0,2);
    mat_assign(H, mat_eye(vec_size), 0,2, 12,14);
    mat_assign(H, vec_to_skewmat(mat_vec_mul(quat_to_dcm(quat_est), mag_ref)), 3,5, 0,2);
    mat_assign(H, mat_eye(vec_size), 3,5, 15,17);
    
    return H;
}


/* 
 * Function to get Phi (Model Step)
 */
std::vector<std::vector<float>> mekf::get_Phi(float delta_t, std::vector<float> gyro_meas, std::vector<float> accel_meas){

    // 
    std::vector<std::vector<float>> F(state_size, std::vector<float>(state_size));
    std::vector<std::vector<float>> Phi(state_size, std::vector<float>(state_size));

    // Build F
    mat_assign(F, mat_num_mul(-1, vec_to_skewmat(gyro_meas)), 0,2, 0,2);
    mat_assign(F, mat_num_mul(-1, mat_eye(vec_size)), 0,2, 9,11);
    mat_assign(F, mat_num_mul(-1, mat_mul(mat_T(quat_to_dcm(quat_est)), vec_to_skewmat(accel_meas))), 3,5, 0,2);
    mat_assign(F, mat_num_mul(-1, mat_T(quat_to_dcm(quat_est))), 3,5, 12,14);
    mat_assign(F, mat_eye(vec_size), 6,8, 3,5);

    // discrete state transition
    Phi = mat_add(mat_eye(state_size), mat_num_mul(delta_t, F));

    return Phi;
}




