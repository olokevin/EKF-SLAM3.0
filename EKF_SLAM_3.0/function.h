#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

//地图行，列
#define map_row 84
#define map_column 5

//里程计行，列
#define odometry_row 18391
#define odometry_column 3

//机器人控制向量噪声参数
#define alpha_1 0.1 //平移噪声参数
#define alpha_2 0.05
#define alpha_3 0.05
#define alpha_4 0.1 //旋转噪声参数

//landmark数量
#define num_landmark 84

//传感器参数
#define sensor_range_max 60
#define sensor_range_min 1
#define sensor_bearing 2*pi

//函数声明

//开辟内存
float ** request_memory(float **array, int array_row, int array_column);

//地图初始化
void read_map_data(float **map);

//里程计初始化
void read_odometry_data(float **odometry);

//联合状态向量初始化
void state_vector_init(matrix *state_vector, float **odometry);

//获取里程计数据
void get_odometry(float **odometry, int i,
	float *theta_0, float *theta_1, float *x_0, float *x_1, float *y_0, float *y_1);

//调整角度
void adjust_angle(float *angle);

//计算控制向量
void calculate_u(float *rot_1, float *trans, float *rot_2,
	float *theta_0, float *theta_1, float *x_0, float *x_1, float *y_0, float *y_1);

//预测联合状态向量
void predict_state_vector(int i, matrix *state_vector, matrix *m_next_state_vector, float *rot_1, float *trans, float *rot_2);

//计算控制空间噪声协方差矩阵Mt
void calculate_Mt(float *rot_1, float *trans, float *rot_2, matrix *Mt);

//计算控制空间到状态空间的雅可比矩阵Vt
void calculate_Vt(float *rot_1, float *trans, matrix *Vt, matrix *state_vector);

//计算雅可比矩阵gt
void calculate_gt(matrix *gt, matrix *state_vector, float *rot_1, float *trans);

//观测模型
matrix do_observation_model(float **map, matrix *next_state_vector, int i);

//产生高斯噪声
matrix make_noise(int i);

//内存扩张
float ** array_extend(float **source, int row, int column, int delta_row, int delta_column);

//更新联合状态向量
void update_state_vector(matrix *next_state_vector, matrix *z, int storage_place);

//计算新landmark雅可比矩阵jGxv
void calculate_jGxv(matrix *jGxv, matrix  *next_state_vector, matrix *z);

//计算新landmark雅可比矩阵jGz
void calculate_jGz(matrix *jGz, matrix  *next_state_vector, matrix *z);

//计算已发现landmark雅可比矩阵H
void calculate_H(matrix *H, matrix  *next_state_vector, matrix *z, int storage_place);