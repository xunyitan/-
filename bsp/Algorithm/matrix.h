#ifndef MATRIX_H
#define MATRIX_H

#include "kalman_filter.h"
#include <stdbool.h>

typedef struct {
	int rows;
	int cols;
	float* data;
	arm_matrix_instance_f32 arm_mat_;
} Matrixf;


typedef struct {
	int rows;
	int cols;
	float data[3 * 3];  // 3x3æÿ’Û
	arm_matrix_instance_f32 arm_mat_;
} Matrixf33;

typedef struct {
	int rows;
	int cols;
	float data[4 * 4];  // 4x4æÿ’Û
	arm_matrix_instance_f32 arm_mat_;
} Matrixf44;

typedef struct {
	int rows;
	int cols;
	float data[3 * 1];  // 3x1œÚ¡ø
	arm_matrix_instance_f32 arm_mat_;
} Matrixf31;

void Matrixf_init(Matrixf *mat,int rols,int cols);
void Matrixf_InitWithData(Matrixf *mat,int rols,int cols,float* data);
void Matrixf_copy(Matrixf* dest, const Matrixf* src);
uint32_t Matrixf_rows(const Matrixf* mat); 
uint32_t Matrixf_cols(const Matrixf* mat);
float* Matrixf_at(Matrixf* mat, int row);
void Matrixf_copy_assign(Matrixf* dest, const Matrixf* src);
void Matrixf_add_assign(Matrixf* mat1, const Matrixf* mat2);
void Matrixf_sub_assign(Matrixf* mat1, const Matrixf* mat2);
void Matrixf_mul_assign_float(Matrixf* mat, float val);
void Matrixf_div_assign_float(Matrixf* mat, float val);
void Matrixf_add(const Matrixf* mat1, const Matrixf* mat2, Matrixf* res);
void Matrixf_sub(const Matrixf* mat1, const Matrixf* mat2, Matrixf* res);
void Matrixf_mul_float(const Matrixf* mat, float val, Matrixf* res);
void Matrixf_float_mul_mat(float val, const Matrixf* mat, Matrixf* res);
void Matrixf_div_float(const Matrixf* mat, float val, Matrixf* res);
void Matrixf_mult(const Matrixf* mat1, const Matrixf* mat2, Matrixf* res);
bool Matrixf_equal(const Matrixf* mat1, const Matrixf* mat2);
void Matrixf_block(const Matrixf* mat, int start_row, int start_col, int block_rows, int block_cols, Matrixf* res);
void Matrixf_row(const Matrixf* mat, int row, Matrixf* res);
void Matrixf_col(const Matrixf* mat, int col, Matrixf* res);
void Matrixf_trans(const Matrixf* mat, Matrixf* res);
float Matrixf_trace(const Matrixf* mat);
float Matrixf_norm(const Matrixf* mat);
void Matrixf_inv(const Matrixf* mat, Matrixf* res);
void Matrixf_zeros(Matrixf* mat);
void Matrixf_ones(Matrixf* mat);
void Matrixf_eye(Matrixf* mat);
void Matrixf_diag(const Matrixf* vec, Matrixf* res);

#endif
