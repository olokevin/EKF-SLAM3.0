#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define pi 3.14159

//定义一个矩阵结构体
typedef struct
{
	float **data;
	int row, column;
} matrix;

//矩阵操作函数声明

//矩阵初始化
matrix matrix_init(int row, int column);

//矩阵元素清零
matrix matrix_clear(matrix *mat);

//矩阵内存释放
matrix matrix_free(matrix *mat);

//矩阵元素赋值
void matrix_set_data(matrix *mat, float **array);

//矩阵深拷贝
//void matrix_copy(matrix *dst, const matrix *src);

//矩阵转置
matrix matrix_transpose(const matrix *mat);

//矩阵加法
matrix matrix_add(const matrix *mat1, const matrix *mat2);

//矩阵减法
matrix matrix_sub(const matrix *mat1, const matrix *mat2);

//矩阵乘法
matrix matrix_multiply(const matrix *mat1, const matrix *mat2);

//矩阵打印
void matrix_print(const matrix *mat);

//矩阵取子矩阵（i1到i2行，j1到j2列）
matrix matrix_submatrix(matrix *source, int i1, int i2, int j1, int j2);

//矩阵求逆
matrix matrix_inverse(const matrix *source);

//生成单位阵
matrix generate_I(int dimension);

//矩阵替换
void matrix_substitude(matrix *source1, int i1, int i2, int j1, int j2, matrix *source2);

//矩阵扩张
void matrix_extend(matrix *source, int delta_row, int delta_column);