#include "matrix.h"

void Matrixf_init(Matrixf *mat,int rols,int cols)
{
	if(!mat)
		return;
	
	mat->rows=rols;
	mat->cols=cols;
	arm_mat_init_f32(&(mat->arm_mat_),rols,cols,mat->data);
}

void Matrixf_InitWithData(Matrixf *mat,int rols,int cols,float* data)
{
	if(!mat)
		return;
	
	Matrixf_init(mat,rols,cols);
	memcpy(mat->data,data,rols*cols*sizeof(float));
	arm_mat_init_f32(&(mat->arm_mat_),rols,cols,mat->data);
}

//对应复制构造函数 Matrixf(const Matrixf<_rows, _cols> &mat)
void Matrixf_copy(Matrixf* dest, const Matrixf* src)
{
	// 检查维度是否匹配
	if (dest->rows != src->rows || dest->cols != src->cols) {
			// 错误处理 - 维度不匹配
			return;
	}
	
	memcpy(dest->data, src->data, src->rows * src->cols * sizeof(float));
	// 重新初始化ARM矩阵结构
	arm_mat_init_f32(&dest->arm_mat_, dest->rows, dest->cols, dest->data);
}

// 对应 uint32_t rows(void) const
uint32_t Matrixf_rows(const Matrixf* mat)
{
	return mat->rows;
}

// 对应 uint32_t cols(void) const  
uint32_t Matrixf_cols(const Matrixf* mat)
{
	return mat->cols;
}

// 对应 float *operator[](const int &row)
float* Matrixf_at(Matrixf* mat, int row)
{
  return &mat->data[row * mat->cols];
}

//对应复制赋值运算符operator
void Matrixf_copy_assign(Matrixf* dest, const Matrixf* src)
{
	memcpy(dest->data, src->data, dest->rows * dest->cols * sizeof(float));
}

// 对应 operator+=
void Matrixf_add_assign(Matrixf* mat1, const Matrixf* mat2)
{
	arm_mat_add_f32(&mat1->arm_mat_, &mat2->arm_mat_, &mat1->arm_mat_);
}

// 对应 operator-=
void Matrixf_sub_assign(Matrixf* mat1, const Matrixf* mat2)
{
	arm_mat_sub_f32(&mat1->arm_mat_, &mat2->arm_mat_, &mat1->arm_mat_);
}

// 对应 operator*=
void Matrixf_mul_assign_float(Matrixf* mat, float val)
{
	arm_mat_scale_f32(&mat->arm_mat_, val, &mat->arm_mat_);
}

// 对应 operator/=
void Matrixf_div_assign_float(Matrixf* mat, float val)
{
	arm_mat_scale_f32(&mat->arm_mat_, 1.f / val, &mat->arm_mat_);
}

// 对应 operator+
void Matrixf_add(const Matrixf* mat1, const Matrixf* mat2, Matrixf* res)
{
	arm_mat_add_f32(&mat1->arm_mat_, &mat2->arm_mat_, &res->arm_mat_);
}

// 对应 operator-
void Matrixf_sub(const Matrixf* mat1, const Matrixf* mat2, Matrixf* res)
{
	arm_mat_sub_f32(&mat1->arm_mat_, &mat2->arm_mat_, &res->arm_mat_);
}

// 对应 operator*(float)
void Matrixf_mul_float(const Matrixf* mat, float val, Matrixf* res)
{
	arm_mat_scale_f32(&mat->arm_mat_, val, &res->arm_mat_);
}

// 对应友元函数 operator*(float, const Matrixf&)
void Matrixf_float_mul_mat(float val, const Matrixf* mat, Matrixf* res)
{
	arm_mat_scale_f32(&mat->arm_mat_, val, &res->arm_mat_);
}

// 对应 operator/(float)
void Matrixf_div_float(const Matrixf* mat, float val, Matrixf* res)
{
	arm_mat_scale_f32(&mat->arm_mat_, 1.f / val, &res->arm_mat_);
}

// 对应矩阵乘法 operator*(const Matrixf&, const Matrixf&)
void Matrixf_mult(const Matrixf* mat1, const Matrixf* mat2, Matrixf* res)
{
	arm_mat_mult_f32(&mat1->arm_mat_, &mat2->arm_mat_, &res->arm_mat_);
}

// 对应 operator==
bool Matrixf_equal(const Matrixf* mat1, const Matrixf* mat2)
{
	for (int i = 0; i < mat1->rows * mat1->cols; i++)
	{
		if (mat1->data[i] != mat2->data[i])
				return false;
	}
	return true;
}

// 对应 block 函数
void Matrixf_block(const Matrixf* mat, int start_row, int start_col, int block_rows, int block_cols, Matrixf* res)
{
    for (int row = start_row; row < start_row + block_rows; row++)
    {
        memcpy(res->data + (row - start_row) * block_cols, 
               mat->data + row * mat->cols + start_col, 
               block_cols * sizeof(float));
    }
}

// 对应 row 函数
void Matrixf_row(const Matrixf* mat, int row, Matrixf* res)
{
    Matrixf_block(mat, row, 0, 1, mat->cols, res);
}

// 对应 col 函数
void Matrixf_col(const Matrixf* mat, int col, Matrixf* res)
{
    Matrixf_block(mat, 0, col, mat->rows, 1, res);
}

// 对应 trans 函数
void Matrixf_trans(const Matrixf* mat, Matrixf* res)
{
    arm_mat_trans_f32(&mat->arm_mat_, &res->arm_mat_);
}

// 对应 trace 函数
float Matrixf_trace(const Matrixf* mat)
{
	float res = 0;
	int min_dim = (mat->rows < mat->cols) ? mat->rows : mat->cols;
	for (int i = 0; i < min_dim; i++)
	{
			res += mat->data[i * mat->cols + i];
	}
	return res;
}

// 对应 norm 函数
float Matrixf_norm(const Matrixf* mat)
{
	Matrixf trans_mat, mult_result;
	Matrixf_trans(mat, &trans_mat);
	Matrixf_mult(&trans_mat, mat, &mult_result);
	return sqrtf(mult_result.data[0]);
}

// 对应 inv 函数
void Matrixf_inv(const Matrixf* mat, Matrixf* res)
{
	if (mat->cols != mat->rows)
	{
			// 返回零矩阵
			memset(res->data, 0, res->rows * res->cols * sizeof(float));
			return;
	}
	
	arm_status status = arm_mat_inverse_f32(&mat->arm_mat_, &res->arm_mat_);
	
	if (status == ARM_MATH_SINGULAR)
	{
			// 返回零矩阵
			memset(res->data, 0, res->rows * res->cols * sizeof(float));
	}
}

// 对应 zeros 静态函数
void Matrixf_zeros(Matrixf* mat)
{
	memset(mat->data, 0, mat->rows * mat->cols * sizeof(float));
}

// 对应 ones 静态函数
void Matrixf_ones(Matrixf* mat)
{
	for (int i = 0; i < mat->rows * mat->cols; i++)
	{
			mat->data[i] = 1.0f;
	}
}

// 对应 eye 静态函数
void Matrixf_eye(Matrixf* mat)
{
	memset(mat->data, 0, mat->rows * mat->cols * sizeof(float));
	int min_dim = (mat->rows < mat->cols) ? mat->rows : mat->cols;
	for (int i = 0; i < min_dim; i++)
	{
			mat->data[i * mat->cols + i] = 1.0f;
	}
}

// 对应 diag 静态函数
void Matrixf_diag(const Matrixf* vec, Matrixf* res)
{
	Matrixf_zeros(res);
	int min_dim = (res->rows < res->cols) ? res->rows : res->cols;
	for (int i = 0; i < min_dim; i++)
	{
			res->data[i * res->cols + i] = vec->data[i];
	}
}

