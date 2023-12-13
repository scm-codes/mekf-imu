#ifndef MEKF_H
#define MEKF_H
#include <vector>

/*
 * perform MEKF algorithm
 * Args:    float delta_t
 *          
 * Returns: vector<float>       
*/

/* State Vector 
 * [0-2] error in quaternion parameter
 * [3-5] error in velocity 
 * [6-8] error in position 
 * [9-11] delta gyro bias 
 * [12-14] delta accel bias
 * [15-17] delta mag bias
*/

class mekf{

private:
    int vec_size;
    int quat_size;
    int state_size; 
    int num_obsv;

    float gyro_var; 
    float accel_var; 
    float mag_var; 

    float gyro_bias_var;
    float accel_bias_var;
    float mag_bias_var; 

    float accel_obsv_var;
    float mag_obsv_var;

    std::vector<float> state_est;
    std::vector<float> quat_est;
    std::vector<std::vector<float>> cov_est;
    std::vector<std::vector<float>> obsv_cov;

    std::vector<float> gyro_bias;
    std::vector<float> accel_bias;
    std::vector<float> mag_bias;

    std::vector<float> accel_ref;
    std::vector<float> mag_ref;

    std::vector<std::vector<float>> get_Q(float delta_t);
    std::vector<std::vector<float>> get_H();
    std::vector<std::vector<float>> get_Phi(float delta_t, std::vector<float> gyro_meas, std::vector<float> accel_meas);
    void init_vars();

public:
    mekf();

    void mekf_step(float delta_t, std::vector<float> gyro_meas, std::vector<float> accel_meas, std::vector<float> mag_meas);
    void set_accel_ref(std::vector<float> ref_vec);
    void set_mag_ref(std::vector<float> ref_vec);
    std::vector<float> get_eul_est();
    std::vector<float> get_quat_est();
    std::vector<float> get_diag_cov_est();
    std::vector<float> get_gvec_est();
    std::vector<std::vector<float>> get_cov_est();

    // ~mekf();
};


#endif