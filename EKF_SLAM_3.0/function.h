#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

//��ͼ�У���
#define map_row 84
#define map_column 5

//��̼��У���
#define odometry_row 18391
#define odometry_column 3

//�����˿���������������
#define alpha_1 0.1 //ƽ����������
#define alpha_2 0.05
#define alpha_3 0.05
#define alpha_4 0.1 //��ת��������

//landmark����
#define num_landmark 84

//����������
#define sensor_range_max 60
#define sensor_range_min 1
#define sensor_bearing 2*pi

//��������

//�����ڴ�
float ** request_memory(float **array, int array_row, int array_column);

//��ͼ��ʼ��
void read_map_data(float **map);

//��̼Ƴ�ʼ��
void read_odometry_data(float **odometry);

//����״̬������ʼ��
void state_vector_init(matrix *state_vector, float **odometry);

//��ȡ��̼�����
void get_odometry(float **odometry, int i,
	float *theta_0, float *theta_1, float *x_0, float *x_1, float *y_0, float *y_1);

//�����Ƕ�
void adjust_angle(float *angle);

//�����������
void calculate_u(float *rot_1, float *trans, float *rot_2,
	float *theta_0, float *theta_1, float *x_0, float *x_1, float *y_0, float *y_1);

//Ԥ������״̬����
void predict_state_vector(int i, matrix *state_vector, matrix *m_next_state_vector, float *rot_1, float *trans, float *rot_2);

//������ƿռ�����Э�������Mt
void calculate_Mt(float *rot_1, float *trans, float *rot_2, matrix *Mt);

//������ƿռ䵽״̬�ռ���ſɱȾ���Vt
void calculate_Vt(float *rot_1, float *trans, matrix *Vt, matrix *state_vector);

//�����ſɱȾ���gt
void calculate_gt(matrix *gt, matrix *state_vector, float *rot_1, float *trans);

//�۲�ģ��
matrix do_observation_model(float **map, matrix *next_state_vector, int i);

//������˹����
matrix make_noise(int i);

//�ڴ�����
float ** array_extend(float **source, int row, int column, int delta_row, int delta_column);

//��������״̬����
void update_state_vector(matrix *next_state_vector, matrix *z, int storage_place);

//������landmark�ſɱȾ���jGxv
void calculate_jGxv(matrix *jGxv, matrix  *next_state_vector, matrix *z);

//������landmark�ſɱȾ���jGz
void calculate_jGz(matrix *jGz, matrix  *next_state_vector, matrix *z);

//�����ѷ���landmark�ſɱȾ���H
void calculate_H(matrix *H, matrix  *next_state_vector, matrix *z, int storage_place);