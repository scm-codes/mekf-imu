#ifndef MEKFMATHOP_H
#define MEKFMATHOP_H
#include <vector>


/* Calculate norm of a given vector */
float vec_norm(std::vector<float> vec);

/* Calculate unit vector of a given vector */
std::vector<float> vec_unit(std::vector<float> vec);

/* Calculate addition of two vectors */
std::vector<float> vec_add(std::vector<float> vec_1, std::vector<float> vec_2);

/* Calculate subtracton of two vectors */
std::vector<float> vec_sub(std::vector<float> vec_1, std::vector<float> vec_2);

/* Concatenate two vectors */
std::vector<float> vec_join(std::vector<float> vec_1, std::vector<float> vec_2);

/* Calculate number times vector */
std::vector<float> vec_num_mul(float num, std::vector<float> vec);

/* Calculate norm of a given quaternion */
float quat_norm(std::vector<float> quat);

/* Calculate unit quaternion of a given quaternion */
std::vector<float> quat_unit(std::vector<float> quat);

/* Calculate multiplication of a two quaternions */
std::vector<float> quat_mul(std::vector<float> quat_1, std::vector<float> quat_2);

/* Calculate direction cosine matrix corresponding to a unit quaternion */
std::vector<std::vector<float>> quat_to_dcm(std::vector<float> quat);

/* Calculate euler angles corresponding to a unit quaternion */
std::vector<float> quat_to_eul(std::vector<float> quat);

/* Perform matrix multiplication */
std::vector<std::vector<float>> mat_mul(std::vector<std::vector<float>> mat_1,std::vector<std::vector<float>> mat_2);

/* Perform matrix and vector multiplication */
std::vector<float> mat_vec_mul(std::vector<std::vector<float>> mat,std::vector<float> vec);

/* Calculates constant times a matrix */
std::vector<std::vector<float>> mat_num_mul(float num, std::vector<std::vector<float>> mat);

/* Perform matrix addition */
std::vector<std::vector<float>> mat_add(std::vector<std::vector<float>> mat_1,std::vector<std::vector<float>> mat_2);

/* Perform matrix subtraction */
std::vector<std::vector<float>> mat_sub(std::vector<std::vector<float>> mat_1,std::vector<std::vector<float>> mat_2);

/* Calculates matrix transpose */
std::vector<std::vector<float>> mat_T(std::vector<std::vector<float>> mat);

/* Creates identity matrix */
std::vector<std::vector<float>> mat_eye(int dim);

/* Assigns one matrix to other */
void mat_assign(std::vector<std::vector<float>> &mat_1, std::vector<std::vector<float>> mat_2, int row_start, int row_end, int col_start, int col_end);

/* Create a skew symmetric matrix from a vector */
std::vector<std::vector<float>> vec_to_skewmat(std::vector<float> vec);

/* Print a vector */
void vec_print(std::vector<float> vec);

/* print a matrix */
void mat_print(std::vector<std::vector<float>> mat);

/* main method for Cholesky decomposition. */
void choldc1(int mat_size, std::vector<std::vector<float>> &mat_temp, std::vector<float> &mat_diag);

/* Inverse of Cholesky decomposition */
std::vector<std::vector<float>> choldcsl(int mat_size, std::vector<std::vector<float>> mat);

/* Matrix inverse using Cholesky decomposition */
std::vector<std::vector<float>> cholsl(std::vector<std::vector<float>> mat);



#endif