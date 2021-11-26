#define _CRT_SECURE_NO_WARNINGS

#include  "matrix_operation.h"
#include  "function.h"

//函数定义

//开辟内存
float ** request_memory(float **array, int array_row, int array_column) {
	int i;

	array = (float**)malloc(sizeof(float*)*array_row);
	if (array == NULL)
	{
		printf("array no memory\n");
		exit(1);
	}
	for (i = 0; i < array_row; i++)
	{
		array[i] = (float*)malloc(sizeof(float)*array_column);
		if (array[i] == NULL)
		{
			printf("array no memory\n");
			exit(1);
		}
		memset(array[i], 0, sizeof(float)*array_column);
	}
	return array;
}

//地图初始化
void read_map_data(float **map) {
	FILE *fp;

	int i, j;

	//打开文件
	if ((fp = fopen("map.txt", "r")) == NULL)
	{
		printf("open error\n");
		exit(1);
	}

	//置文件首部
	fseek(fp, 0L, 0);

	//读入数据
	while (!feof(fp))
		for (i = 0; i < map_row; i++)
			for (j = 0; j < map_column; j++)
				fscanf(fp, "%f,", &map[i][j]);

	////输出显示
	//for (i = 0; i < map_row; i++)
	//{
	//	for (j = 0; j < map_column; j++)
	//		j == map_column - 1 ? printf("%f", map[i][j]) : printf("%f,", map[i][j]);
	//	printf("\n");
	//}

	//关闭文件
	fclose(fp);
}

//里程计初始化
void read_odometry_data(float **odometry) {
	FILE *fp;

	int i, j;

	//打开文件
	if ((fp = fopen("odometry.txt", "r")) == NULL)
	{
		printf("open error\n");
		exit(1);
	}

	//置文件首部
	fseek(fp, 0L, 0);

	//读入数据
	while (!feof(fp))
		for (i = 0; i < odometry_row; i++)
			for (j = 0; j < odometry_column; j++)
				fscanf(fp, "%f,", &odometry[i][j]);

	////输出显示
	//for (i = 0; i < odometry_row; i++)
	//{
	//	for (j = 0; j < odometry_column; j++)
	//		j == odometry_column - 1 ? printf("%f", odometry[i][j]) : printf("%f,", odometry[i][j]);
	//	printf("\n");
	//}

	//关闭文件
	fclose(fp);
}

//联合状态向量初始化
void state_vector_init(matrix *state_vector, float **odometry) {
	state_vector->data[0][0] = *(*(odometry + 0) + 0);
	state_vector->data[1][0] = *(*(odometry + 0) + 1);
	state_vector->data[2][0] = *(*(odometry + 0) + 2);
}

//获取里程计数据
void get_odometry(float **odometry, int i,
	float *theta_0, float *theta_1, float *x_0, float *x_1, float *y_0, float *y_1) {
	//里程计角度theta
	*theta_0 = *(*(odometry + i) + 2);
	*theta_1 = *(*(odometry + i + 1) + 2);

	//里程计横坐标x
	*x_0 = *(*(odometry + i) + 0);
	*x_1 = *(*(odometry + i + 1) + 0);

	//里程计纵坐标y
	*y_0 = *(*(odometry + i) + 1);
	*y_1 = *(*(odometry + i + 1) + 1);
}

//调整角度
void adjust_angle(float *angle) {
	if (*angle > pi) {
		*angle = *angle - 2 * pi;
	}
	else if (*angle < -pi) {
		*angle = *angle + 2 * pi;
	}
}

//计算控制向量
/*
sigma = p0 ~ p1
rot_1 = sigma - theta_o
*/
void calculate_u(float *rot_1, float *trans, float *rot_2,
	float *theta_0, float *theta_1, float *x_0, float *x_1, float *y_0, float *y_1) {
	*rot_1 = atan2f(*y_1 - *y_0, *x_1 - *x_0) - *theta_0;
	adjust_angle(rot_1);
	*trans = sqrtf(powf(*x_1 - *x_0, 2) + powf(*y_1 - *y_0, 2));
	*rot_2 = *theta_1 - *theta_0 - *rot_1;
	adjust_angle(rot_2);
}

//预测联合状态向量
//dx=
void predict_state_vector(int i, matrix *state_vector, matrix *next_state_vector, float *rot_1, float *trans, float *rot_2) {

	float delta_x = *trans * cosf(state_vector->data[2][0] + *rot_1);
	float delta_y = *trans * sinf(state_vector->data[2][0] + *rot_1);
	float delta_theta = *rot_1 + *rot_2;
	adjust_angle(&delta_theta);

	//printf("delta_x : %f	delta_y : %f	delta_theta : %f	\n", delta_x, delta_y, delta_theta);

	/*printf("第 %d 次预测前state_vector：\n", i);
	matrix_print(state_vector);
	printf("\n");*/

	next_state_vector->data[0][0] = state_vector->data[0][0] + delta_x;
	next_state_vector->data[1][0] = state_vector->data[1][0] + delta_y;
	next_state_vector->data[2][0] = state_vector->data[2][0] + delta_theta;
	adjust_angle(&next_state_vector->data[2][0]);
}

//计算控制空间噪声协方差矩阵Mt
void calculate_Mt(float *rot_1, float *trans, float *rot_2, matrix *Mt) {
	Mt->data[0][0] = alpha_1 * powf(*rot_1, 2) + alpha_2 * powf(*trans, 2);
	Mt->data[1][1] = alpha_3 * powf(*trans, 2) + alpha_4 * powf(*rot_1, 2) + alpha_4 * powf(*rot_2, 2);
	Mt->data[2][2] = alpha_1 * powf(*rot_2, 2) + alpha_2 * powf(*trans, 2);
}

//计算控制空间到状态空间的雅可比矩阵Vt
void calculate_Vt(float *rot_1, float *trans, matrix *Vt, matrix *state_vector) {
	Vt->data[0][0] = -*trans * sinf(state_vector->data[2][0] + *rot_1);
	Vt->data[1][0] = *trans * cosf(state_vector->data[2][0] + *rot_1);
	Vt->data[2][0] = 1;
	Vt->data[0][1] = cosf(state_vector->data[2][0] + *rot_1);
	Vt->data[1][1] = sinf(state_vector->data[2][0] + *rot_1);
	Vt->data[2][2] = 1;
}

//计算gt
void calculate_gt(matrix *gt, matrix *state_vector, float *rot_1, float *trans) {
	gt->data[0][2] = -*trans * sinf(state_vector->data[2][0] + *rot_1);
	gt->data[1][2] = -*trans * cosf(state_vector->data[2][0] + *rot_1);
}

//观测模型
matrix do_observation_model(float **map, matrix *next_state_vector, int i) {
	matrix z = matrix_init(2,1);
	float delta_x = *(*(map + i) + 1) - next_state_vector->data[0][0];
	float delta_y = *(*(map + i) + 2) - next_state_vector->data[1][0];
	float range = sqrtf(powf(delta_x, 2) + powf(delta_y, 2));
	float angle = atan2f(delta_y, delta_x) - next_state_vector->data[2][0];
	adjust_angle(&angle);
	z.data[0][0] = range;
	z.data[1][0] = angle;
	return z;
}

float GaussRandNorm() {
	float U, V, z;
	U = rand() / (RAND_MAX + 1.0);
	V = rand() / (RAND_MAX + 1.0);
	z = sqrtf(-2.0 * logf(U)) * sinf(2.0 * pi * V);
	return z;
}

//产生高斯噪声
matrix make_noise(int n) {
	matrix noise = matrix_init(n, 1);
	
	noise.data[0][0] = GaussRandNorm();
	noise.data[1][0] = GaussRandNorm() / 10;

	return noise;
}

//内存扩张
float ** array_extend(float **source, int row, int column, int delta_row, int delta_column) {
	int i;

	//
	source = (float **)realloc(source, (row + delta_row) * sizeof(float *));

	for (i = 0; i < row + delta_row; i++)
	{
		source[i] = (float*)malloc(sizeof(float) * (column + delta_column));

		memset(source[i], 0, sizeof(float) * (column + delta_column));
	}
	return source;
}

//更新联合状态向量
void update_state_vector(matrix *next_state_vector, matrix *z, int storage_place) {
	next_state_vector->data[storage_place][0] = next_state_vector->data[0][0]
		+ z->data[0][0] * cosf(z->data[1][0] + next_state_vector->data[2][0]);
	next_state_vector->data[storage_place + 1][0] = next_state_vector->data[1][0]
		+ z->data[0][0] * sinf(z->data[1][0] + next_state_vector->data[2][0]);
}

//计算新landmark雅可比矩阵jGxv
void calculate_jGxv(matrix *jGxv, matrix  *next_state_vector, matrix *z){
	jGxv->data[0][0] = 1;
	jGxv->data[1][1] = 1;
	jGxv->data[0][2] = -z->data[0][0] * sinf(next_state_vector->data[2][0] + z->data[1][0]);
	jGxv->data[1][2] = z->data[0][0] * cosf(next_state_vector->data[2][0] + z->data[1][0]);
}

//计算新landmark雅可比矩阵jGz
void calculate_jGz(matrix *jGz, matrix  *next_state_vector, matrix *z) {
	jGz->data[0][0] = cosf(next_state_vector->data[2][0] + z->data[1][0]);
	jGz->data[0][1] = -z->data[0][0] * sinf(next_state_vector->data[2][0] + z->data[1][0]);
	jGz->data[1][0] = sinf(next_state_vector->data[2][0] + z->data[1][0]);
	jGz->data[1][1] = z->data[0][0] * cosf(next_state_vector->data[2][0] + z->data[1][0]);
}

//计算已发现landmark雅可比矩阵H
void calculate_H(matrix *H, matrix  *next_state_vector, matrix *z, int storage_place) {
	//
	float delta_x = z->data[0][0] * cosf(next_state_vector->data[2][0] + z->data[1][0]);
	float delta_y = z->data[0][0] * sinf(z->data[1][0] + next_state_vector->data[2][0]);
	//
	H->data[0][0] = -delta_x / z->data[0][0];
	H->data[1][0] = delta_y / powf(z->data[0][0], 2);
	//
	H->data[0][1] = -delta_y / z->data[0][0];
	H->data[1][1] = -delta_x / powf(z->data[0][0], 2);
	//
	H->data[1][2] = -1;
	//
	H->data[0][storage_place] = -H->data[0][0];
	H->data[1][storage_place] = -H->data[1][0];
	//
	H->data[0][storage_place + 1] = -H->data[0][1];
	H->data[1][storage_place + 1] = -H->data[1][1];
}