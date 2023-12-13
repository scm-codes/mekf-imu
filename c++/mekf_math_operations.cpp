#include "mekf_math_operations.h"
#include <vector>
#include <cmath>
#include <iostream>

/* Calculate norm of a given vector
 * Args:    vector<float> vector
 * Returns: <float> norm
*/
float vec_norm(std::vector<float> vec){
    float norm = sqrt(pow(vec[0],2) + pow(vec[1],2) + pow(vec[2],2)); 
    return norm;
}


/* Calculate unit vector of a given vector
 * Args:    vector<float> vector
 * Returns: vector<float> norm(vector)
*/
std::vector<float> vec_unit(std::vector<float> vec){
    int vec_size = 3;
    std::vector<float> vec_mod(vec_size);
    
    float norm = vec_norm(vec);

    vec_mod[0] = vec[0] / norm;
    vec_mod[1] = vec[1] / norm;
    vec_mod[2] = vec[2] / norm;

    return vec_mod;
}

/* Calculate number times vector 
 * Args:    <float> num, vector<float> vec
 * Returns: vector<float> vec_mod
*/
std::vector<float> vec_num_mul(float num, std::vector<float> vec){
    int vec_size = vec.size();
    std::vector<float> vec_mod(vec_size);

    for (int ii = 0; ii < vec_size; ii++){
        vec_mod[ii] = num*vec[ii];
    }

    return vec_mod;
}


/* Calculate addition of two vectors 
 * Args:    vector<float> vec_1, vector<float> vec_2
 * Returns: vector<float> vec_3
*/
std::vector<float> vec_add(std::vector<float> vec_1, std::vector<float> vec_2){
    int vec_size = vec_1.size();
    std::vector<float> vec_3(vec_size);

    for (int ii = 0; ii < vec_size; ii++){
        vec_3[ii] = vec_1[ii] + vec_2[ii];
    }

    return vec_3;
}

/* Concatenate two vectors 
 * Args:    vector<float> vec_1, vector<float> vec_2
 * Returns: vector<float> vec_3
*/
std::vector<float> vec_join(std::vector<float> vec_1, std::vector<float> vec_2){

    int vec_1_size = vec_1.size();
    int vec_2_size = vec_2.size();

    std::vector<float> vec_3(vec_1_size + vec_2_size);

    for(int ii = 0; ii < vec_1_size; ii++){
        vec_3[ii] = vec_1[ii];
    }

    for(int ii = 0; ii < vec_2_size; ii++){
        vec_3[ii+vec_1_size] = vec_2[ii];
    }

    return vec_3;
}


/* Calculate subtracton of two vectors 
 * Args:    vector<float> vec_1, vector<float> vec_2
 * Returns: vector<float> vec_3
*/
std::vector<float> vec_sub(std::vector<float> vec_1, std::vector<float> vec_2){
    int vec_size = vec_1.size();
    std::vector<float> vec_3(vec_size);

    for (int ii = 0; ii < vec_size; ii++){
        vec_3[ii] = vec_1[ii] - vec_2[ii];
    }
    return vec_3;

}


/* Calculate norm of a given quaternion
 * Args:    vector<float> vector
 * Returns: <float> norm
*/
float quat_norm(std::vector<float> quat){
    float norm = sqrt(pow(quat[0],2) + pow(quat[1],2) + pow(quat[2],2) + pow(quat[3],2)); 
    return norm;
}


/* Calculate unit quaternion of a given quaternion
 * Args:    vector<float> quaternion
 * Returns: vector<float> norm(quaternion)
*/
std::vector<float> quat_unit(std::vector<float> quat){
    int quat_size = 4;
    std::vector<float> quat_mod(quat_size);
    
    float norm = quat_norm(quat);

    quat_mod[0] = quat[0] / norm;
    quat_mod[1] = quat[1] / norm;
    quat_mod[2] = quat[2] / norm;
    quat_mod[3] = quat[3] / norm;

    return quat_mod;
}


/* Calculate multiplication of a two quaternions
 * Args:    vector<float> quaternion, vector<float> quaternion
 * Returns: vector<float> quaternion
*/
std::vector<float> quat_mul(std::vector<float> quat_1, std::vector<float> quat_2){
    int quat_size = 4;
    std::vector<float> quat(quat_size);

    quat[0] = quat_2[0]*quat_1[0] - quat_2[1]*quat_1[1] - quat_2[2]*quat_1[2] - quat_2[3]*quat_1[3];
    quat[1] = quat_2[0]*quat_1[1] + quat_2[1]*quat_1[0] - quat_2[2]*quat_1[3] + quat_2[3]*quat_1[2];
    quat[2] = quat_2[0]*quat_1[2] + quat_2[1]*quat_1[3] + quat_2[2]*quat_1[0] - quat_2[3]*quat_1[1];
    quat[3] = quat_2[0]*quat_1[3] - quat_2[1]*quat_1[2] + quat_2[2]*quat_1[1] + quat_2[3]*quat_1[0];

    return quat;
}


/* Calculate direction cosine corresponding to a unit quaternion
 * Args:    vector<float> quaternion
 * Returns: vector<vector<float>>
*/

std::vector<std::vector<float>> quat_to_dcm(std::vector<float> quat){

    int vec_size = 3;
    std::vector<std::vector<float>> dcm(vec_size, std::vector<float>(vec_size));

    // first column
    dcm[0][0] = 2 * (quat[0]*quat[0] + quat[1]*quat[1]) - 1;
    dcm[1][0] = 2 * (quat[1]*quat[2] - quat[0]*quat[3]);
    dcm[2][0] = 2 * (quat[1]*quat[3] + quat[0]*quat[2]);

    // second column
    dcm[0][1] = 2 * (quat[1]*quat[2] + quat[0]*quat[3]);
    dcm[1][1] = 2 * (quat[0]*quat[0] + quat[2]*quat[2]) - 1;
    dcm[2][1] = 2 * (quat[2]*quat[3] - quat[0]*quat[1]);

    // third column
    dcm[0][2] = 2 * (quat[1]*quat[3] - quat[0]*quat[2]);
    dcm[1][2] = 2 * (quat[2]*quat[3] + quat[0]*quat[1]);
    dcm[2][2] = 2 * (quat[0]*quat[0] + quat[3]*quat[3]) - 1;

    return dcm;
}



/* Calculate euler angles corresponding to a unit quaternion
 * Args:    vector<float> quaternion
 * Returns: vector<float>
*/
std::vector<float> quat_to_eul(std::vector<float> quat){

    int vec_size = 3;
    std::vector<float> eul(vec_size);

    // yaw - pitch - roll
    eul[0] = std::atan2( 2 * (quat[0]*quat[3] + quat[1]*quat[2]), (1 - 2*(quat[2]*quat[2] + quat[3]*quat[3])) );
    eul[1] = std::asin( 2 * (quat[0]*quat[2] - quat[3]*quat[1]) );
    eul[2] = std::atan2( 2 * (quat[0]*quat[1] + quat[2]*quat[3]), (1 - 2*(quat[1]*quat[1] + quat[2]*quat[2])) );

    return eul;
}


/* Perform matrix multiplication 
 * Args:    vector<vector<float>> mat_1, vector<vector<float>> mat_2
 * Returns: vector<vector<float>> mat_3 
*/
std::vector<std::vector<float>> mat_mul(std::vector<std::vector<float>> mat_1,std::vector<std::vector<float>> mat_2){

    int row_size_mat_3, col_size_mat_3;
    row_size_mat_3 = mat_1.size();
    col_size_mat_3 = mat_2[0].size();
    std::vector<std::vector<float>> mat_3(row_size_mat_3, std::vector<float>(col_size_mat_3));

    for(int ii = 0; ii < mat_1.size();  ii++){
        for(int kk = 0; kk < mat_2[0].size(); kk++){
            float term = 0;
            for (int jj = 0; jj < mat_1[0].size(); jj++){
                term += mat_1[ii][jj]*mat_2[jj][kk];
            }
            mat_3[ii][kk] = term;
        }
    }

    return mat_3;
}


/* Perform matrix addition
 * Args:    vector<vector<float>> mat_1, vector<vector<float>> mat_2
 * Returns: vector<vector<float>> mat_3 
*/
std::vector<std::vector<float>> mat_add(std::vector<std::vector<float>> mat_1,std::vector<std::vector<float>> mat_2){

    int row_size_mat_3, col_size_mat_3;
    row_size_mat_3 = mat_1.size();
    col_size_mat_3 = mat_1[0].size();
    std::vector<std::vector<float>> mat_3(row_size_mat_3, std::vector<float>(col_size_mat_3));

    for(int ii = 0; ii < row_size_mat_3 ;  ii++){
        for(int jj = 0; jj < col_size_mat_3; jj++){
            mat_3[ii][jj] = mat_1[ii][jj] + mat_2[ii][jj];
        }
    }

    return mat_3;
}

/* Perform matrix subtraction
 * Args:    vector<vector<float>> mat_1, vector<vector<float>> mat_2
 * Returns: vector<vector<float>> mat_3 
*/
std::vector<std::vector<float>> mat_sub(std::vector<std::vector<float>> mat_1,std::vector<std::vector<float>> mat_2){

    int row_size_mat_3, col_size_mat_3;
    row_size_mat_3 = mat_1.size();
    col_size_mat_3 = mat_1[0].size();
    std::vector<std::vector<float>> mat_3(row_size_mat_3, std::vector<float>(col_size_mat_3));

    for(int ii = 0; ii < row_size_mat_3 ;  ii++){
        for(int jj = 0; jj < col_size_mat_3; jj++){
            mat_3[ii][jj] = mat_1[ii][jj] - mat_2[ii][jj];
        }
    }

    return mat_3;
}


/* Perform matrix and vector multiplication 
 * Args:    vector<vector<float>> mat, vector<float> vec
 * Returns: vector<float> vec_out
*/
std::vector<float> mat_vec_mul(std::vector<std::vector<float>> mat,std::vector<float> vec){

    int vec_out_size = mat.size();
    std::vector<float> vec_out(vec_out_size);

    for(int ii = 0; ii < mat.size();  ii++){
        float term = 0;
        for(int jj = 0; jj < mat[0].size(); jj++){
            term += mat[ii][jj]*vec[jj];
        }
        vec_out[ii] = term;
    }

    return vec_out;
}

/* Creates identity matrix 
 * Args:    <int> dim
 * Returns: vector<vector<float>> eye
*/
std::vector<std::vector<float>> mat_eye(int dim){

    std::vector<std::vector<float>> eye(dim, std::vector<float>(dim));

    for(int ii = 0; ii < dim;  ii++){
        eye[ii][ii] = 1;
    }

    return eye;
}

/* Calculates constant times a matrix 
 * Args:   <float> num, vector<vector<float>> mat
 * Returns: 
*/
std::vector<std::vector<float>> mat_num_mul(float num, std::vector<std::vector<float>> mat){

    std::vector<std::vector<float>> mat_mod(mat.size(), std::vector<float>(mat[0].size()));

    for(int ii = 0; ii < mat.size();  ii++){
        for (int jj = 0; jj < mat[0].size(); jj++){
            mat_mod[ii][jj] = num * mat[ii][jj];
        }
    }

    return mat_mod; 
}

/* Calculates matrix transpose 
 * Args:    vector<vector<float>> mat
 * Returns: vector<vector<float>> mat_t
*/
std::vector<std::vector<float>> mat_T(std::vector<std::vector<float>> mat){

    int row_size, col_size;
    row_size = mat[0].size();
    col_size = mat.size();

    std::vector<std::vector<float>> mat_t(row_size, std::vector<float>(col_size));

    for(int ii = 0; ii < row_size;  ii++){
        for(int jj = 0; jj < col_size; jj++){
            mat_t[ii][jj] = mat[jj][ii];
        }
    }

    return mat_t;
}


/* Assigns one matrix to other
 * Args:    
*/
void mat_assign(std::vector<std::vector<float>> &mat_1, std::vector<std::vector<float>> mat_2, int row_start, int row_end, int col_start, int col_end){


    int kk, ll;
    kk = ll = 0;

    for(int ii = row_start; ii <= row_end; ii++){
        for(int jj = col_start; jj <= col_end; jj++){
            mat_1[ii][jj] = mat_2[kk][ll]; 
            ll += 1;
        }
        ll = 0;
        kk += 1;
    }

}


/* Create a skew symmetric matrix from a vector
 * Args: vector<float> vec 
 * Returns: vector<vector<float>> skewmat
 */

std::vector<std::vector<float>> vec_to_skewmat(std::vector<float> vec){

    int vec_size = 3;
    std::vector<std::vector<float>> skewmat(vec_size, std::vector<float>(vec_size));

    skewmat[0][1] = -vec[2];
    skewmat[1][0] = vec[2];
    skewmat[0][2] = vec[1];
    skewmat[2][0] = -vec[1];
    skewmat[1][2] = -vec[0];
    skewmat[2][1] = vec[0];

    return skewmat;
}


/* Print a vector 
 * Args: vector<float> vec
*/
void vec_print(std::vector<float> vec){

    for(int ii = 0; ii < vec.size();  ii++){
        std::cout << vec[ii] << " ";
    }
    std::cout << "\n" << std::endl;

}


/* Print a matrix
 * Args: vector<vector<float>> matrix
*/
void mat_print(std::vector<std::vector<float>> mat){

    for(int ii = 0; ii < mat.size();  ii++){
        for(int jj = 0; jj < mat[0].size(); jj++){
            std::cout << mat[ii][jj] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}


/*Cholesky Inverse Section 
 * NOTE: works only for symmetric matrices
 * Cholesky-decomposition matrix-inversion code, adapated from
 * http://jean-pierre.moreau.pagesperso-orange.fr/Cplus/choles_cpp.txt 
 */

/* main method for Cholesky decomposition */
void choldc1(int mat_size, std::vector<std::vector<float>> &mat_temp, std::vector<float> &mat_diag) {

    float sum;

    for (int ii = 0; ii < mat_size; ii++) {
        for (int jj = ii; jj < mat_size; jj++) {
            sum = mat_temp[ii][jj];
            for (int kk = ii - 1; kk >= 0; kk--) {
                sum -= mat_temp[ii][kk] * mat_temp[jj][kk];
            }
            if (ii == jj) {
                if (sum <= 0) {
                    printf(" covariance matrix is not positive definite!\n");
                }
                mat_diag[ii] = sqrt(sum);
            }
            else {
                mat_temp[jj][ii] = sum / mat_diag[ii];
            }
        }
    }
}


/* Inverse of Cholesky decomposition */
std::vector<std::vector<float>> choldcsl(int mat_size, std::vector<std::vector<float>> mat) {

    float sum;
    std::vector<std::vector<float>> mat_temp;
    std::vector<float> mat_temp_diag(mat_size);

    mat_temp = mat;

    choldc1(mat_size, mat_temp, mat_temp_diag);

    for (int ii = 0; ii < mat_size; ii++) {
        mat_temp[ii][ii] = 1 / mat_temp_diag[ii];
        for (int jj = ii + 1; jj < mat_size; jj++) {
            sum = 0;
            for (int kk = ii; kk < jj; kk++) {
                sum -= mat_temp[jj][kk] * mat_temp[kk][ii];
            }
            mat_temp[jj][ii] = sum / mat_temp_diag[jj];
        }
    }
        
    return mat_temp;
}


/* Matrix inverse using Cholesky decomposition
 * NOTE: works only for symmetric matrices

 */
std::vector<std::vector<float>> cholsl(std::vector<std::vector<float>> mat) {

    int mat_size = mat.size();
    std::vector<std::vector<float>> mat_i(mat_size, std::vector<float>(mat_size));

    mat_i = choldcsl(mat_size, mat);
    

    for (int ii = 0; ii < mat_size; ii++) {
        for (int jj = ii + 1; jj < mat_size; jj++) {
            mat_i[ii][jj] = 0.0;
        }
    } 

    for (int ii = 0; ii < mat_size; ii++) {
        mat_i[ii][ii] *= mat_i[ii][ii];
        for (int kk = ii + 1; kk < mat_size; kk++) {
            mat_i[ii][ii] += mat_i[kk][ii] * mat_i[kk][ii];
        }
        for (int jj = ii + 1; jj < mat_size; jj++) {
            for (int kk = jj; kk < mat_size; kk++) {
                mat_i[ii][jj] += mat_i[kk][ii] * mat_i[kk][jj];
            }
        }
    }
    for (int ii = 0; ii < mat_size; ii++) {
        for (int jj = 0; jj < ii; jj++) {
            mat_i[ii][jj] = mat_i[jj][ii];
        }
    }

    return mat_i;
}
