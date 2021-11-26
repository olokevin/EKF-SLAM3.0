#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include  "matrix_operation.h"
#include  "function.h"

int main()
{
	float start, finish; //��ʼʱ��,����ʱ��
	float begin, end;
	float duration = 0;

	//�ڴ�ռ��
	int used_memory = 0;

	//ѭ������
	int i = 0;

	float **map = NULL;
	map = request_memory(map, map_row, map_column);

	//��txt�ļ��ж�ȡ��ͼ����
	read_map_data(map);

	float **odometry = NULL;
	odometry = request_memory(odometry, odometry_row, odometry_column);

	//��txt�ļ��ж�ȡ��̼�����
	read_odometry_data(odometry);

	//state_vector = (x, y, theta, m_x1, m_y1, m_x2, my_2, ......)
	matrix state_vector = matrix_init(3, 1);
	state_vector_init(&state_vector, odometry);
	/*printf("state_vector_init : \n");
	matrix_print(&state_vector);*/

	//next_state_vector = (x, y, theta, m_x1, m_y1, m_x2, my_2, ......)
	matrix next_state_vector = matrix_init(state_vector.row, 1);

	//Э����
	matrix covariance = matrix_init(3, 3);
	/*printf("covariance_init : \n");
	matrix_print(&covariance);*/

	//��̼ƽǶ�theta
	float theta_0;
	float theta_1;

	//��̼ƺ�����x
	float x_0;
	float x_1;

	//��̼�������y
	float y_0;
	float y_1;

	//��������
	float rot_1;
	float trans;
	float rot_2;

	//������ƿռ�����Э�������Mt����Щ���õ��Ǹ�˹���������˴����������������С�й�
	matrix Mt = matrix_init(3, 3);

	//������ƿռ䵽״̬�ռ���ſɱȾ���Vt
	matrix Vt = matrix_init(3, 3);

	//�����ſɱȾ���gt ״̬���·��̵�G = I + F^T gt F
	matrix gt = matrix_init(3, 3);

	//��������Э�������Qt
	float sigma_r = 0.02;
	float sigma_phi = 0.001;

	matrix Qt = matrix_init(2, 2);
	Qt.data[0][0] = sigma_r * sigma_r;
	Qt.data[1][1] = sigma_phi * sigma_phi;
	/*printf("Qt : \n");
	matrix_print(&Qt);
	printf("\n");*/

	//landmark_information = {landmark_index, new_landmark_flag}
	int landmark_information[2] = {-1,0};
	
	//�۲�����
	matrix z = matrix_init(2, 1);
	matrix z_predict = matrix_init(2, 1);

	//��landmark�ſɱȾ���
	//jGxv: �۲�ռ�(x,y)<-������״̬�ռ�(x,y,��)
	matrix jGxv = matrix_init(2, 3);  
	//jGz: �۲�ռ�(x,y)<-landmark״̬�ռ�(x,y)
	matrix jGz = matrix_init(2, 2);

	//
	/*FILE *fpWrite1 = fopen("predict_state_vector.txt", "w");
	if (fpWrite1 == NULL)
	{
		return 0;
	}*/

	FILE *fpWrite2 = fopen("correction_state_vector.txt", "w");
	if (fpWrite2 == NULL)
	{
		return 0;
	}

	FILE *fpWrite3 = fopen("state_vector_dimension.txt", "w");
	if (fpWrite3 == NULL)
	{
		return 0;
	}

	//��ʼ��ʱ
	start = (float)clock();

	for (i = 0; i < odometry_row - 1; i++)
	{		
		//��ʼ��ʱ
		begin = (float)clock();

		//��ȡ��̼�����
			//���εĻ����˦ȣ�x��y(״̬����)
		get_odometry(odometry, i, &theta_0, &theta_1, &x_0, &x_1, &y_0, &y_1);
		/*printf("theta_0 : %f	theta_1 : %f	x_0 : %f	x_1 : %f	y_0 : %f	y_1 : %f	\n", theta_0, theta_1, x_0, x_1, y_0, y_1);
		printf("\n");*/

		//�����������
		calculate_u(&rot_1, &trans, &rot_2, &theta_0, &theta_1, &x_0, &x_1, &y_0, &y_1);
		/*printf("rot_1 : %f	trans : %f	rot_2 : %f	\n", rot_1, trans, rot_2);
		printf("\n");*/

		//�������˶�����>����Ԥ�⡪У��
		if (rot_1 + rot_2 < powf(10, -8) && trans < powf(10, -8))
		{
			continue;
		}

		//Ԥ��

		//Ԥ������״̬����
		predict_state_vector(i, &state_vector, &next_state_vector, &rot_1, &trans, &rot_2);
		/*printf("�� %d ��Ԥ��next_state_vector��\n", i);
		matrix_print(&next_state_vector);
		printf("\n");*/

		//������ƿռ�����Э�������Mt
		calculate_Mt(&rot_1, &trans, &rot_2, &Mt);
		/*printf("Mt : \n");
		matrix_print(&Mt);
		printf("\n");*/

		//������ƿռ䵽״̬�ռ���ſɱȾ���Vt
		calculate_Vt(&rot_1, &trans, &Vt, &state_vector);
		/*printf("Vt : \n");
		matrix_print(&Vt);
		printf("\n");*/

		//����Rt = V Mt Vt^T
		matrix Vt_Mt = matrix_multiply(&Vt, &Mt);
		matrix Vt_T = matrix_transpose(&Vt);
		matrix Rt = matrix_multiply(&Vt_Mt, &Vt_T);
		/*printf("Rt : \n");
		matrix_print(&Rt);
		printf("\n");*/

		//�����ſɱȾ���gt
		calculate_gt(&gt, &state_vector, &rot_1, &trans);
		/*printf("gt : \n");
		matrix_print(&gt);
		printf("\n");*/

		//Ԥ��Э�������next_covariance_vv
		matrix gt_T = matrix_transpose(&gt);
		//ȡ�Ӿ��� �õ�1~3 * 1~3�� ״̬Э������� ��vv
		matrix covariance_vv = matrix_submatrix(&covariance, 1, 3, 1, 3);
		matrix gt_covariance_vv = matrix_multiply(&gt, &covariance_vv);
		matrix gt_covariance_vv_gt_T = matrix_multiply(&gt_covariance_vv, &gt_T);
		matrix next_covariance_vv = matrix_add(&gt_covariance_vv_gt_T, &Rt);
		/*printf("next_covariance_vv : \n");
		matrix_print(&next_covariance_vv);
		printf("\n");*/

		matrix next_covariance = matrix_init(covariance.row, covariance.column);

		if (covariance.column > 3)
		{
			//Ԥ��Э�������next_covariance_vm vehicle-map
			matrix covariance_vm = matrix_submatrix(&covariance, 1, 3, 4, covariance.column);
			matrix next_covariance_vm = matrix_multiply(&gt, &covariance_vm);
			
			//Ԥ��Э�������next_covariance_vm_T
			matrix next_covariance_vm_T = matrix_transpose(&next_covariance_vm);

			//Ԥ��Э�������next_covariance_mm
			matrix next_covariance_mm = matrix_submatrix(&covariance, 4, covariance.row, 4, covariance.column);

			//Ԥ��Э�������next_covariance
			matrix_substitude(&next_covariance, 1, 3, 1, 3, &next_covariance_vv);
			matrix_substitude(&next_covariance, 1, 3, 4, covariance.column, &next_covariance_vm);
			matrix_substitude(&next_covariance, 4, covariance.row, 1, 3, &next_covariance_vm_T);
			matrix_substitude(&next_covariance, 4, covariance.row, 4, covariance.column, &next_covariance_mm);

			//
			matrix_free(&covariance_vm);
			matrix_free(&next_covariance_vm);
			matrix_free(&next_covariance_vm_T);
			matrix_free(&next_covariance_mm);
		}
		else
		{
			matrix_substitude(&next_covariance, 1, 3, 1, 3, &next_covariance_vv);
		}

		//fprintf(fpWrite1, "%f %f\n", next_state_vector.data[0][0], next_state_vector.data[1][0]);

		//
		matrix_free(&Vt_Mt);
		matrix_free(&Vt_T);
		matrix_free(&Rt);
		matrix_free(&gt_T);
		matrix_free(&gt_covariance_vv);
		matrix_free(&gt_covariance_vv_gt_T);
		matrix_free(&covariance_vv);
		matrix_free(&next_covariance_vv);

		//////////////////////////////////////////Ԥ�⡪��У�� �ֽ���//////////////////////////////////////////

		//У��

		//����ÿһ��·��
		landmark_information[0] = -1; //landmark_index		��·���signature
		landmark_information[1] = 0; //new_landmark_flag	�Ƿ�Ϊ��landmark	

		int j;
		for (j = 0; j < num_landmark; j++) {
			/*
				12 delta
				13 q
				14 z_t^i
			*/
			matrix z_predict_temp = do_observation_model(map, &next_state_vector, j);
			/*printf("�� %d ��z_predict : \n", j);
			matrix_print(&z_predict_temp);
			printf("\n");*/

			matrix noise = make_noise(2);
			noise.data[0][0] = noise.data[0][0] * sigma_r;
			noise.data[1][0] = noise.data[1][0] * sigma_phi;
			/*printf("�� %d ������ : \n", i);
			matrix_print(&noise);
			printf("\n");*/

			matrix z_temp = matrix_add(&z_predict_temp, &noise);
			adjust_angle(&z_temp.data[1][0]);
			/*printf("�� %d ��z_temp : \n", j);
			matrix_print(&z_temp);
			printf("\n");*/

			//��·��
			if (z_temp.data[0][0] >= sensor_range_min && z_temp.data[0][0] <= sensor_range_max 
				&& z_temp.data[1][0] <= sensor_bearing && *(*(map + j) + 3) == 0)
			{
				landmark_information[1] = 1; //new_landmark_flag
				landmark_information[0] = j; //landmark_index
				*(*(map + j) + 3) = 1;
				
				//�����滻
				matrix_substitude(&z, 1, 2, 1, 1, &z_temp);
				matrix_substitude(&z_predict, 1, 2, 1, 1, &z_predict_temp);
				
				//
				matrix_free(&noise);
				matrix_free(&z_predict_temp);
				matrix_free(&z_temp);

				break;
			}
			//��·��
			else if(z_temp.data[0][0] >= sensor_range_min && z_temp.data[0][0] <= sensor_range_max
				&& z_temp.data[1][0] <= sensor_bearing)
			{
				landmark_information[0] = j; //landmark_index
				matrix_substitude(&z, 1, 2, 1, 1, &z_temp);
				matrix_substitude(&z_predict, 1, 2, 1, 1, &z_predict_temp);

				//
				matrix_free(&noise);
				matrix_free(&z_predict_temp);
				matrix_free(&z_temp);

				break;
			}

			//
			matrix_free(&noise);
			matrix_free(&z_predict_temp);
			matrix_free(&z_temp);
		}

		if (landmark_information[0] != -1) //landmark_index
		{
			//printf("���ֵ� %d ��z : \n", landmark_information[0]);
			/*matrix_print(&z);
			printf("\n");*/
			if (landmark_information[1] == 1) //new_landmark_flag
			{
				//printf("���·��ֵ�landmark ! \n");
				*(*(map + landmark_information[0]) + 4) = next_state_vector.row; //storage_place
				
				////�����ͼ��ʾ
				//for (int row = 0; row < map_row; row++)
				//{
				//	for (int column = 0; column < map_column; column++)
				//		j == map_column - 1 ? printf("%f", map[row][column]) : printf("%f,", map[row][column]);
				//	printf("\n");
				//}
				//printf("\n");

				//
				matrix_extend(&next_state_vector, 2, 0);

				/*printf("��ά���next_state_vector�� \n");
				matrix_print(&next_state_vector);
				printf("\n");*/

				matrix_extend(&state_vector, 2, 0);
				
				//����״̬���������¹۲⵽��landmark��(x,y)����
				update_state_vector(&next_state_vector, &z, *(*(map + landmark_information[0]) + 4));
				/*printf("next_state_vector�� \n");
				matrix_print(&next_state_vector);
				printf("\n");*/

				calculate_jGxv(&jGxv, &next_state_vector, &z);
				/*printf("jGxv�� \n");
				matrix_print(&jGxv);
				printf("\n");*/

				calculate_jGz(&jGz, &next_state_vector, &z);
				/*printf("jGz�� \n");
				matrix_print(&jGz);
				printf("\n");*/

				//��չЭ�������
				//��ʼ��һ����λ����
				matrix M = generate_I(next_state_vector.row);

				//jGxv: �۲�ռ�(x,y)<-������״̬�ռ�(x,y,��)
				matrix_substitude(&M, next_state_vector.row - 1, next_state_vector.row, 
					1, 3, &jGxv);
				//jGz: �۲�ռ�(x,y)<-landmark״̬�ռ�(x,y)
				matrix_substitude(&M, next_state_vector.row - 1, next_state_vector.row, 
					next_state_vector.row - 1, next_state_vector.row, &jGz);
				/*printf("M�� \n");
				matrix_print(&M);
				printf("\n");*/

				matrix M_T = matrix_transpose(&M);

				//��landmark��Ӧ��Э�����ʼ��ΪQt
				matrix_extend(&next_covariance, 2, 2);
				matrix_substitude(&next_covariance, next_covariance.row - 1, next_covariance.row, 
					next_covariance.row - 1, next_covariance.row, &Qt);
				/*printf("next_covariance�� \n");
				matrix_print(&next_covariance);
				printf("\n");*/
				
				//��չЭ������󣬶�Ӧ��ΪM �� M^T	(MΪ��ʱ���󣬶�Ӧ״̬���������е�G)
				matrix M_next_covariance = matrix_multiply(&M, &next_covariance);
				matrix M_next_covariance_M_T = matrix_multiply(&M_next_covariance, &M_T);
				matrix_substitude(&next_covariance, 1, next_covariance.row, 
					1, next_covariance.row, &M_next_covariance_M_T);
				/*printf("next_covariance�� \n");
				matrix_print(&next_covariance);
				printf("\n");*/

				//
				matrix_extend(&covariance, 2, 2);

				//
				matrix_free(&M);
				matrix_free(&M_T);
				matrix_free(&M_next_covariance);
				matrix_free(&M_next_covariance_M_T);
			}
			else
			{
				//printf("���ѷ��ֵ�landmark ! \n");
				
				//���ݼ����ᵽ��H �۲��״̬�����ĸ���
				matrix H = matrix_init(2, next_state_vector.row);
				calculate_H(&H, &next_state_vector, &z, *(*(map + landmark_information[0]) + 4));
				/*printf("H�� \n");
				matrix_print(&H);
				printf("\n");*/

				matrix H_T = matrix_transpose(&H);

				//
				matrix innovation = matrix_sub(&z, &z_predict);
				adjust_angle(&innovation.data[1][0]);

				//
				matrix H_next_covariance = matrix_multiply(&H, &next_covariance);
				matrix H_next_covariance_H_T = matrix_multiply(&H_next_covariance, &H_T);
				matrix S = matrix_add(&H_next_covariance_H_T, &Qt);
				/*printf("S�� \n");
				matrix_print(&S);
				printf("\n");*/

				matrix S_inv = matrix_inverse(&S);
				/*printf("S_inv�� \n");
				matrix_print(&S_inv);
				printf("\n");*/

				//���㿨��������
				matrix next_covariance_H_T = matrix_multiply(&next_covariance, &H_T);
				matrix K = matrix_multiply(&next_covariance_H_T, &S_inv);
				/*printf("K�� \n");
				matrix_print(&K);
				printf("\n");*/

				matrix K_T = matrix_transpose(&K);

				//
				matrix K_innovation = matrix_multiply(&K, &innovation);
				matrix next_state_vector_temp = matrix_add(&next_state_vector, &K_innovation);
				matrix_substitude(&next_state_vector, 1, next_state_vector.row, 
					1, next_state_vector.column, &next_state_vector_temp);

				//
				matrix K_S = matrix_multiply(&K, &S);
				matrix K_S_K_T = matrix_multiply(&K_S, &K_T);
				matrix next_covariance_temp = matrix_sub(&next_covariance, &K_S_K_T);
				matrix_substitude(&next_covariance, 1, next_covariance.row, 
					1, next_covariance.column, &next_covariance_temp);
				/*printf("next_covariance�� \n");
				matrix_print(&next_covariance);
				printf("\n");*/

				//
				matrix_free(&H);
				matrix_free(&H_T);
				matrix_free(&innovation);
				matrix_free(&H_next_covariance);
				matrix_free(&H_next_covariance_H_T);
				matrix_free(&S);
				matrix_free(&S_inv);
				matrix_free(&next_covariance_H_T);
				matrix_free(&K);
				matrix_free(&K_T);
				matrix_free(&K_innovation);
				matrix_free(&next_state_vector_temp);
				matrix_free(&K_S);
				matrix_free(&K_S_K_T);
				matrix_free(&next_covariance_temp);
			}
		}

		//
		matrix_substitude(&state_vector, 1, next_state_vector.row, 1, next_state_vector.column, &next_state_vector);
		matrix_substitude(&covariance, 1, next_covariance.row, 1, next_covariance.column, &next_covariance);

		//������ʱ
		end = (float)clock();
		duration = duration + (end - begin);
		//��ӡ����ʱ�䣬��λ������
		/*printf("�ۼ�����ʱ�䣺 \n");
		printf("%.4f����", duration);
		printf("\n");*/

		//
		fprintf(fpWrite2, "%f %f\n", next_state_vector.data[0][0], next_state_vector.data[1][0]);
		fprintf(fpWrite3, "%d %d %d %d %f\n", i, next_state_vector.row, (next_state_vector.row - 3) / 2, next_state_vector.row * next_state_vector.row * 4 + next_state_vector.row * 4 + 16, duration);
		
		//
		
		matrix_free(&next_covariance);

		
	}

	//��ʱ����
	finish = (float)clock();

	//��ӡ����ʱ�䣬��λ������
	printf("����ʱ�䣺 \n");
	printf("%.4f����", (finish - start));
	printf("\n");

	//����״̬����ά��
	printf("����״̬����ά����%d \n", next_state_vector.row);

	printf("next_state_vector�� \n");
	matrix_print(&next_state_vector);
	printf("\n");

	/*printf("covariance�� \n");
	matrix_print(&covariance);
	printf("\n");*/

	//�ͷŵ�ͼ�ڴ�
	for (i = 0; i < map_row; i++)
		free(map[i]);
	free(map);

	//�ͷ���̼��ڴ�
	for (i = 0; i < odometry_row; i++)
		free(odometry[i]);
	free(odometry);

	//
	matrix_free(&next_state_vector);
	matrix_free(&state_vector);
	matrix_free(&covariance);
	matrix_free(&Mt);
	matrix_free(&Vt);
	matrix_free(&gt);
	matrix_free(&Qt);
	matrix_free(&z);
	matrix_free(&z_predict);
	matrix_free(&jGxv);
	matrix_free(&jGz);

	//
	//fclose(fpWrite1);
	fclose(fpWrite2);
	fclose(fpWrite3);
}