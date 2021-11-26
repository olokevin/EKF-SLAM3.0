#include  "matrix_operation.h"

//矩阵操作函数定义

//矩阵初始化
matrix matrix_init(int row, int column) {
	matrix result;
	if (row <= 0 || column <= 0)
	{
		printf("error, in create_mat: row <= 0||column<=0\n");
		exit(1);
	}
	if (row > 0 && column > 0)
	{
		result.row = row;
		result.column = column;
		result.data = (float **)malloc(row * sizeof(float *));
		if (result.data == NULL)
		{
			printf("error, in create_mat: mat.data==NULL");
			exit(1);
		}
		int i;
		for (i = 0; i < row; i++)
		{
			*(result.data + i) = (float *)malloc(column * sizeof(float));//再分配每行的指针
			if (result.data[i] == NULL)
			{
				printf("error, in create_mat: mat.data==NULL");
				exit(1);
			}
		}
		matrix_clear(&result);
	}
	return result;
}

//矩阵元素清零
matrix matrix_clear(matrix *mat) {
	int i, j;

	for (i = 0; i < mat->row; i++)
	{
		for (j = 0; j < mat->column; j++)
		{
			mat->data[i][j] = 0;
		}
	}
}

//矩阵内存释放
matrix matrix_free(matrix *mat) {
	int i;

	for (i = 0; i < mat->row; i++)

		//释放行
		free(mat->data[i]);

	//释放头指针
	free(mat->data);
}

//矩阵元素赋值
void matrix_set_data(matrix *mat, float **array) {
	int i, j;

	for (i = 0; i < mat->row; i++)
	{
		for (j = 0; j < mat->column; j++)
		{
			mat->data[i][j] = *(*(array + i) + j);
		}
	}
}

//矩阵深拷贝
//void matrix_copy(matrix *dst, const matrix *src) {
//	
//	if (dst->row == src->row && dst->column == src->column) {
//		int i, j;
//
//		for (i = 0; i < dst->row; i++)
//		{
//			for (j = 0; j < dst->column; j++)
//			{
//				dst->data[i][j] = src->data[i][j];
//			}
//		}
//	}
//	else {
//		printf("error, in mat_copy: matrix dimension error !\n");
//		exit(1);
//	}
//}

//矩阵转置
matrix matrix_transpose(const matrix *mat) {
	matrix result = matrix_init(mat->column, mat->row);

	int i, j;

	for (i = 0; i < mat->row; i++)
	{
		for (j = 0; j < mat->column; j++)
		{
			result.data[j][i] = mat->data[i][j];
		}
	}

	return result;
}

//矩阵加法
matrix matrix_add(const matrix *mat1, const matrix *mat2) {
	matrix mat = matrix_init(mat1->row, mat1->column);
	if (mat1->row == mat2->row && mat1->column == mat2->column) {
		int i, j;

		for (i = 0; i < mat1->row; i++)
		{
			for (j = 0; j < mat1->column; j++)
				mat.data[i][j] = mat1->data[i][j] + mat2->data[i][j];
		}
		return mat;
	}
	else {
		printf("error, in add_mat: matrix dimension error !\n");
		exit(1);
	}
}

//矩阵减法
matrix matrix_sub(const matrix *mat1, const matrix *mat2) {
	matrix mat = matrix_init(mat1->row, mat1->column);
	if (mat1->row == mat2->row && mat1->column == mat2->column) {
		int i, j;

		for (i = 0; i < mat1->row; i++)
		{
			for (j = 0; j < mat1->column; j++)
				mat.data[i][j] = mat1->data[i][j] - mat2->data[i][j];
		}
		return mat;
	}
	else {
		printf("error, in add_mat: matrix dimension error !\n");
		exit(1);
	}
}

//矩阵乘法
matrix matrix_multiply(const matrix *mat1, const matrix *mat2) {
	matrix mat = matrix_init(mat1->row, mat2->column);

	if (mat1->column != mat2->row)
	{
		printf("error,In mult_mat: matrix dimension error !\n");
		exit(1);
	}
	else
	{
		matrix_clear(&mat);
		int i, j;
		for (i = 0; i < mat1->row; i++)
		{
			for (j = 0; j < mat2->column; j++)
			{
				int m;
				for (m = 0; m < mat1->column; m++)
				{
					mat.data[i][j] += mat1->data[i][m] * mat2->data[m][j];
				}
			}
		}
	}
	return mat;
}

//矩阵打印
void matrix_print(const matrix *mat) {
	int i = 0;
	int j = 0;

	//
	for (i = 0; i < mat->row; i++)
	{
		for (j = 0; j < mat->column; j++)
		{
			printf("%12f ", mat->data[i][j]);
		}
		printf("\n");
	}
	printf("\n");
}

//矩阵取子矩阵（i1到i2行，j1到j2列）
matrix matrix_submatrix(matrix *source, int i1, int i2, int j1, int j2) {
	if (i1 <= i2 && j1 <= j2 && i2 <= source->row && j2 <= source->column) {
		matrix result = matrix_init(i2 - i1 + 1, j2 - j1 + 1);

		//
		int i = 0;
		int j = 0;

		//
		for (i = i1 - 1; i <= i2 - 1; i++)
		{
			for (j = j1 - 1; j <= j2 - 1; j++)
			{
				result.data[i - i1 + 1][j - j1 + 1] = source->data[i][j];
			}
		}

		//
		return result;
	}
	else {
		printf("error,In matrix_submatrix: submatrix dimension error !\n");
		exit(1);
	}
}

//矩阵求逆
matrix matrix_inverse(const matrix *source) {
	matrix result = matrix_init(2, 2);
	float det = source->data[0][0] * source->data[1][1] - source->data[0][1] * source->data[1][0];
	result.data[0][0] = source->data[1][1] / det;
	result.data[1][1] = source->data[0][0] / det;
	result.data[0][1] = -source->data[0][1] / det;
	result.data[1][0] = -source->data[1][0] / det;
	return result;
}

//生成单位阵
matrix generate_I(int dimension) {
	matrix mat = matrix_init(dimension, dimension);

	//
	int i;

	for (i = 0; i < dimension; i++)
	{
		mat.data[i][i] = 1;
	}

	return mat;
}

//矩阵替换
void matrix_substitude(matrix *source1, int i1, int i2, int j1, int j2, const matrix *source2) {
	//
	int i = 0;
	int j = 0;

	//
	for (i = i1 - 1; i <= i2 - 1; i++)
	{
		for (j = j1 - 1; j <= j2 - 1; j++)
		{
			source1->data[i][j] = source2->data[i - i1 + 1][j - j1 + 1];
		}
	}
}

//矩阵扩张
void matrix_extend(matrix *source, int delta_row, int delta_column) {
	matrix temp = matrix_init(source->row, source->column);
	matrix_substitude(&temp, 1, source->row, 1, source->column, source);
	source->data = array_extend(source->data, source->row, source->column, delta_row, delta_column);
	source->row = source->row + delta_row;
	source->column = source->column + delta_column;
	matrix_substitude(source, 1, temp.row, 1, temp.column, &temp);
	//
	matrix_free(&temp);
}