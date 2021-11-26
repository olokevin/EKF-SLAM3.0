#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define pi 3.14159

//����һ������ṹ��
typedef struct
{
	float **data;
	int row, column;
} matrix;

//���������������

//�����ʼ��
matrix matrix_init(int row, int column);

//����Ԫ������
matrix matrix_clear(matrix *mat);

//�����ڴ��ͷ�
matrix matrix_free(matrix *mat);

//����Ԫ�ظ�ֵ
void matrix_set_data(matrix *mat, float **array);

//�������
//void matrix_copy(matrix *dst, const matrix *src);

//����ת��
matrix matrix_transpose(const matrix *mat);

//����ӷ�
matrix matrix_add(const matrix *mat1, const matrix *mat2);

//�������
matrix matrix_sub(const matrix *mat1, const matrix *mat2);

//����˷�
matrix matrix_multiply(const matrix *mat1, const matrix *mat2);

//�����ӡ
void matrix_print(const matrix *mat);

//����ȡ�Ӿ���i1��i2�У�j1��j2�У�
matrix matrix_submatrix(matrix *source, int i1, int i2, int j1, int j2);

//��������
matrix matrix_inverse(const matrix *source);

//���ɵ�λ��
matrix generate_I(int dimension);

//�����滻
void matrix_substitude(matrix *source1, int i1, int i2, int j1, int j2, matrix *source2);

//��������
void matrix_extend(matrix *source, int delta_row, int delta_column);