#include<stdio.h>
#include<iostream>
#include<vector>
#include "mekf.h"
#include "mekf_math_operations.h"

int main(){

//     int vec_size = 3;
//     int quat_size = 4;

//     std::vector<float> vec(vec_size);
//     std::vector<float> quat(quat_size);

//     // vec
//     vec[0] = 1; vec[1] = 2; vec[2] = 3;

//     // quat
//     quat[0] = 1; quat[1] = 0.5; quat[2] = 0.5; quat[3] = 0.5;


//     // test vec norm
//     float vnorm = vec_norm(vec);
//     // std::cout << vnorm << std::endl;

//     // test unit vector
//     vec = vec_unit(vec);
//     // std::cout << vec[0] << " " << vec[1] << " " << " " << vec[2] << " " << std::endl;

//     // test quat norm
//     float qnorm = quat_norm(quat);
//     // std::cout << qnorm << std::endl;

//     // test unit quaternion
//     quat = quat_unit(quat);
//     // std::cout << " Quat " << std::endl;
//     // std::cout << quat[0] << " " << quat[1] << " " << quat[2] << " " << quat[3] << std::endl;

//      // test quaternion multiplication
//     std::vector<float> quat_1(quat_size);
//     std::vector<float> quat_2(quat_size);
//     std::vector<float> quat_mul_res(quat_size);

//     quat_1 = quat;
//     quat_2 = quat;
//     quat_mul_res = quat_mul(quat_1, quat_2);
//     // std::cout << quat_mul_res[0] << " " << quat_mul_res[1] << " " << quat_mul_res[2] << " " << quat_mul_res[3] << std::endl;

//     // test quat_to_rotm
//     std::vector<std::vector<float>> mat;
//     mat = quat_to_dcm(quat);
//     //mat_print(mat);

//     // test mat multiplication
//     std::vector<std::vector<float>> mat_1{
//             {1, 0, 0},
//             {0, 1, 0},
//             {1, 2, 3},
//             {0, 1, 2}
//     };

//     std::vector<std::vector<float>> mat_2{
//             {1, 0, 0, 1},
//             {0, 1, 0, 5},
//             {1, 2, 3, 8}
//     };

//     std::vector<std::vector<float>> mat_3;
//     mat_3 = mat_mul(mat_1,mat_2);
//     // mat_print(mat_3);

//     // test matrix transpose
//     std::vector<std::vector<float>> mat_3_T;
//     mat_3_T = mat_T(mat_3);
//     // mat_print(mat_3_T);

//     // test matrix times vector
//     std::vector<float> vec_test;
//     vec_test = mat_vec_mul(mat,vec);
//     // vec_print(vec_test);

//     // test quaternion to euler
//     std::vector<float> eul_test;
//     eul_test = quat_to_eul(quat);
//     // vec_print(eul_test);

//     // add two vectors
//     std::vector<float> vec_res;  
//     vec_res = vec_add(vec_test, eul_test);
//     // vec_print(vec_res);
//     std::vector<std::vector<float>> skewmat;
//     skewmat = vec_to_skewmat(vec_res);
//     // mat_print(skewmat);

//     // test vec concatenation 
//     std::vector<float> vec_tot;
//     vec_tot = vec_join(vec_test, eul_test);
//     // vec_print(vec_tot);

//     // test inverse of matrix
//     std::vector<std::vector<float>> mat_4{
//             {1, 0, 0},
//             {0, 1, 0},
//             {0, 0, 1}
//     };
        
//     std::vector<std::vector<float>> mat_5{
//             {5, -1, -1, -1},
//             {-1, 5, -1, -1},
//             {-1, -1, 5, -1},
//             {-1, -1, -1, 5}
//     };

//     std::vector<std::vector<float>> mat_6{
//             {2, 1, 0, 0, 0, 0},
//             {1, 1, 0, 0, 0, 0},
//             {0, 0, 3, 0, 0, 0},
//             {0, 0, 0, 6, 1, 0},
//             {0, 0, 0, 1, 8, 2},
//             {0, 0, 0, 0, 2, 10},
//     };

//     std::vector<std::vector<float>> mat_4_inv;
//     std::vector<std::vector<float>> mat_5_inv;
//     std::vector<std::vector<float>> mat_6_inv;

//     mat_4_inv = cholsl(mat_4);
//     // mat_print(mat_4_inv);
//     mat_5_inv = cholsl(mat_5);
//     // mat_print(mat_5_inv);
//     mat_6_inv = cholsl(mat_6);
//     // mat_print(mat_6_inv);
    

//     // test identity matrix
//     std::vector<std::vector<float>> mat_7;
//     int mat_7_dim = 7;
//     mat_7 = mat_eye(mat_7_dim);
//     // mat_print(mat_7);

//     // test matrix assign
//     mat_assign(mat_7, mat_6, 0, 5, 0, 5);
//     // mat_print(mat_7);


//     // test const times matrx
//     float num = 2;
//     std::vector<std::vector<float>> mat_8;
//     mat_8 = mat_num_mul(num, mat_eye(num));
//     // mat_print(mat_8);

//     // test vec times num
//     std::vector<float> vec_4 = {1, 2, 3, 4};
//     vec_4 = vec_num_mul(num, vec_4);
//     // vec_print(vec_4);

  
    // // test mekf class
    float delta_t;
    std::vector<float> gyro_meas;
    std::vector<float> accel_meas; 
    std::vector<float> mag_meas;

    // set reference
    /* all ref's are unit vectors 
     * NOTE: ON IMU 
     * accelrometer, gyro - have the same frame
     * magnetometer - has y and z axis flipped on IMU 
     * so while reading magnetometer values fild the y and z. 
     * Example: If reading is (1,2,3) -> (1, -2, -3)
     * (1, -2, -3) is consistent with IMU frame
     */
    std::vector<float> accel_ref = {0, 0, 1}; 
    std::vector<float> mag_ref = {-0.137304567512409, -0.0397013206863993, -0.989732924013335};

    mekf mekf_obj;

    // setting the reference vectors
    /* Since all vectors are computed in NED frame 
     * accel_ref = remain constant = {0, 0, 1}
     * but
     * mag_ref and sun_ref are supposed to be updated frequntely
     * use mag_model and sun_model and use 
     * set_mag_ref, set_sun_ref to set the vectors in the loop
    */
    mekf_obj.set_accel_ref(accel_ref); 
    mekf_obj.set_mag_ref(mag_ref);

    // step - 1
    delta_t = 0.0360; // in s
    gyro_meas = {0.00872664625997165, -0.0221656815003280, 0.0401425727958696}; // in rad/s

    // normalised unit vectors
    // all measurement vectors are normalised (risky but should work)
    accel_meas = {0.000485493646654694, -0.00193206655301358, 0.999998015705408};
    mag_meas = {-0.184689292605859, -0.0221627151127031, -0.982547036663172};
    mekf_obj.mekf_step(delta_t, gyro_meas, accel_meas, mag_meas);

    std::cout << "get euler" << std::endl;
    vec_print(mekf_obj.get_eul_est());

    std::cout << "get quaternon" << std::endl;
    vec_print(mekf_obj.get_quat_est());

    std::cout << "get graity vector" << std::endl;
    vec_print(mekf_obj.get_gvec_est());

    std::cout << "get diag cov" << std::endl;
    vec_print(mekf_obj.get_diag_cov_est());

    std::cout << "get cov" << std::endl;
    mat_print(mekf_obj.get_cov_est());

    std::cout << "end of step - 1 \n" << std::endl;

    

    // step - 2
    delta_t = 0.0360; // in s
    gyro_meas = {0.00261799387799149, 0.0263544717051144, -0.0312413936106985}; // in rad/s

    // normalized unit vectors 
    accel_meas = {0.0204657352935528, 0.00243473398909161, 0.999787590315711};
    mag_meas = {-0.137262851048817, -0.0397339831983416, -0.989737399667783};
    mekf_obj.mekf_step(delta_t, gyro_meas, accel_meas, mag_meas);

    std::cout << "get euler" << std::endl;
    vec_print(mekf_obj.get_eul_est());

    std::cout << "get quaternon" << std::endl;
    vec_print(mekf_obj.get_quat_est());

    std::cout << "get graity vector" << std::endl;
    vec_print(mekf_obj.get_gvec_est());

    std::cout << "get diag cov" << std::endl;
    vec_print(mekf_obj.get_diag_cov_est());

    std::cout << "get cov" << std::endl;
    mat_print(mekf_obj.get_cov_est());

    std::cout << "end of step - 2 \n" << std::endl;

    return 0;
}