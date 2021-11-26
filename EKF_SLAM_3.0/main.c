#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include  "matrix_operation.h"
#include  "function.h"

int main()
{
	float start, finish; //开始时间,结束时间
	float begin, end;
	float duration = 0;

	//内存占用
	int used_memory = 0;

	//循环变量
	int i = 0;

	float **map = NULL;
	map = request_memory(map, map_row, map_column);

	//从txt文件中读取地图坐标
	read_map_data(map);

	float **odometry = NULL;
	odometry = request_memory(odometry, odometry_row, odometry_column);

	//从txt文件中读取里程计数据
	read_odometry_data(odometry);

	//state_vector = (x, y, theta, m_x1, m_y1, m_x2, my_2, ......)
	matrix state_vector = matrix_init(3, 1);
	state_vector_init(&state_vector, odometry);
	/*printf("state_vector_init : \n");
	matrix_print(&state_vector);*/

	//next_state_vector = (x, y, theta, m_x1, m_y1, m_x2, my_2, ......)
	matrix next_state_vector = matrix_init(state_vector.row, 1);

	//协方差
	matrix covariance = matrix_init(3, 3);
	/*printf("covariance_init : \n");
	matrix_print(&covariance);*/

	//里程计角度theta
	float theta_0;
	float theta_1;

	//里程计横坐标x
	float x_0;
	float x_1;

	//里程计纵坐标y
	float y_0;
	float y_1;

	//控制向量
	float rot_1;
	float trans;
	float rot_2;

	//计算控制空间噪声协方差矩阵Mt，有些人用的是高斯白噪声，此处噪声与控制向量大小有关
	matrix Mt = matrix_init(3, 3);

	//计算控制空间到状态空间的雅可比矩阵Vt
	matrix Vt = matrix_init(3, 3);

	//计算雅可比矩阵gt 状态更新方程的G = I + F^T gt F
	matrix gt = matrix_init(3, 3);

	//测量噪声协方差矩阵Qt
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
	
	//观测向量
	matrix z = matrix_init(2, 1);
	matrix z_predict = matrix_init(2, 1);

	//新landmark雅可比矩阵
	//jGxv: 观测空间(x,y)<-机器人状态空间(x,y,θ)
	matrix jGxv = matrix_init(2, 3);  
	//jGz: 观测空间(x,y)<-landmark状态空间(x,y)
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

	//开始计时
	start = (float)clock();

	for (i = 0; i < odometry_row - 1; i++)
	{		
		//开始计时
		begin = (float)clock();

		//获取里程计数据
			//两次的机器人θ，x，y(状态向量)
		get_odometry(odometry, i, &theta_0, &theta_1, &x_0, &x_1, &y_0, &y_1);
		/*printf("theta_0 : %f	theta_1 : %f	x_0 : %f	x_1 : %f	y_0 : %f	y_1 : %f	\n", theta_0, theta_1, x_0, x_1, y_0, y_1);
		printf("\n");*/

		//计算控制向量
		calculate_u(&rot_1, &trans, &rot_2, &theta_0, &theta_1, &x_0, &x_1, &y_0, &y_1);
		/*printf("rot_1 : %f	trans : %f	rot_2 : %f	\n", rot_1, trans, rot_2);
		printf("\n");*/

		//机器人运动——>进入预测—校正
		if (rot_1 + rot_2 < powf(10, -8) && trans < powf(10, -8))
		{
			continue;
		}

		//预测

		//预测联合状态向量
		predict_state_vector(i, &state_vector, &next_state_vector, &rot_1, &trans, &rot_2);
		/*printf("第 %d 次预测next_state_vector：\n", i);
		matrix_print(&next_state_vector);
		printf("\n");*/

		//计算控制空间噪声协方差矩阵Mt
		calculate_Mt(&rot_1, &trans, &rot_2, &Mt);
		/*printf("Mt : \n");
		matrix_print(&Mt);
		printf("\n");*/

		//计算控制空间到状态空间的雅可比矩阵Vt
		calculate_Vt(&rot_1, &trans, &Vt, &state_vector);
		/*printf("Vt : \n");
		matrix_print(&Vt);
		printf("\n");*/

		//计算Rt = V Mt Vt^T
		matrix Vt_Mt = matrix_multiply(&Vt, &Mt);
		matrix Vt_T = matrix_transpose(&Vt);
		matrix Rt = matrix_multiply(&Vt_Mt, &Vt_T);
		/*printf("Rt : \n");
		matrix_print(&Rt);
		printf("\n");*/

		//计算雅可比矩阵gt
		calculate_gt(&gt, &state_vector, &rot_1, &trans);
		/*printf("gt : \n");
		matrix_print(&gt);
		printf("\n");*/

		//预测协方差矩阵next_covariance_vv
		matrix gt_T = matrix_transpose(&gt);
		//取子矩阵 得到1~3 * 1~3的 状态协方差矩阵 ∑vv
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
			//预测协方差矩阵next_covariance_vm vehicle-map
			matrix covariance_vm = matrix_submatrix(&covariance, 1, 3, 4, covariance.column);
			matrix next_covariance_vm = matrix_multiply(&gt, &covariance_vm);
			
			//预测协方差矩阵next_covariance_vm_T
			matrix next_covariance_vm_T = matrix_transpose(&next_covariance_vm);

			//预测协方差矩阵next_covariance_mm
			matrix next_covariance_mm = matrix_submatrix(&covariance, 4, covariance.row, 4, covariance.column);

			//预测协方差矩阵next_covariance
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

		//////////////////////////////////////////预测——校正 分界线//////////////////////////////////////////

		//校正

		//对于每一个路标
		landmark_information[0] = -1; //landmark_index		该路标的signature
		landmark_information[1] = 0; //new_landmark_flag	是否为新landmark	

		int j;
		for (j = 0; j < num_landmark; j++) {
			/*
				12 delta
				13 q
				14 z_t^i
			*/
			matrix z_predict_temp = do_observation_model(map, &next_state_vector, j);
			/*printf("第 %d 个z_predict : \n", j);
			matrix_print(&z_predict_temp);
			printf("\n");*/

			matrix noise = make_noise(2);
			noise.data[0][0] = noise.data[0][0] * sigma_r;
			noise.data[1][0] = noise.data[1][0] * sigma_phi;
			/*printf("第 %d 个噪声 : \n", i);
			matrix_print(&noise);
			printf("\n");*/

			matrix z_temp = matrix_add(&z_predict_temp, &noise);
			adjust_angle(&z_temp.data[1][0]);
			/*printf("第 %d 个z_temp : \n", j);
			matrix_print(&z_temp);
			printf("\n");*/

			//新路标
			if (z_temp.data[0][0] >= sensor_range_min && z_temp.data[0][0] <= sensor_range_max 
				&& z_temp.data[1][0] <= sensor_bearing && *(*(map + j) + 3) == 0)
			{
				landmark_information[1] = 1; //new_landmark_flag
				landmark_information[0] = j; //landmark_index
				*(*(map + j) + 3) = 1;
				
				//矩阵替换
				matrix_substitude(&z, 1, 2, 1, 1, &z_temp);
				matrix_substitude(&z_predict, 1, 2, 1, 1, &z_predict_temp);
				
				//
				matrix_free(&noise);
				matrix_free(&z_predict_temp);
				matrix_free(&z_temp);

				break;
			}
			//旧路标
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
			//printf("发现第 %d 个z : \n", landmark_information[0]);
			/*matrix_print(&z);
			printf("\n");*/
			if (landmark_information[1] == 1) //new_landmark_flag
			{
				//printf("是新发现的landmark ! \n");
				*(*(map + landmark_information[0]) + 4) = next_state_vector.row; //storage_place
				
				////输出地图显示
				//for (int row = 0; row < map_row; row++)
				//{
				//	for (int column = 0; column < map_column; column++)
				//		j == map_column - 1 ? printf("%f", map[row][column]) : printf("%f,", map[row][column]);
				//	printf("\n");
				//}
				//printf("\n");

				//
				matrix_extend(&next_state_vector, 2, 0);

				/*printf("扩维后的next_state_vector： \n");
				matrix_print(&next_state_vector);
				printf("\n");*/

				matrix_extend(&state_vector, 2, 0);
				
				//更新状态向量，将新观测到的landmark的(x,y)加入
				update_state_vector(&next_state_vector, &z, *(*(map + landmark_information[0]) + 4));
				/*printf("next_state_vector： \n");
				matrix_print(&next_state_vector);
				printf("\n");*/

				calculate_jGxv(&jGxv, &next_state_vector, &z);
				/*printf("jGxv： \n");
				matrix_print(&jGxv);
				printf("\n");*/

				calculate_jGz(&jGz, &next_state_vector, &z);
				/*printf("jGz： \n");
				matrix_print(&jGz);
				printf("\n");*/

				//扩展协方差矩阵
				//初始化一个单位矩阵
				matrix M = generate_I(next_state_vector.row);

				//jGxv: 观测空间(x,y)<-机器人状态空间(x,y,θ)
				matrix_substitude(&M, next_state_vector.row - 1, next_state_vector.row, 
					1, 3, &jGxv);
				//jGz: 观测空间(x,y)<-landmark状态空间(x,y)
				matrix_substitude(&M, next_state_vector.row - 1, next_state_vector.row, 
					next_state_vector.row - 1, next_state_vector.row, &jGz);
				/*printf("M： \n");
				matrix_print(&M);
				printf("\n");*/

				matrix M_T = matrix_transpose(&M);

				//新landmark对应的协方差初始化为Qt
				matrix_extend(&next_covariance, 2, 2);
				matrix_substitude(&next_covariance, next_covariance.row - 1, next_covariance.row, 
					next_covariance.row - 1, next_covariance.row, &Qt);
				/*printf("next_covariance： \n");
				matrix_print(&next_covariance);
				printf("\n");*/
				
				//扩展协方差矩阵，对应的为M ∑ M^T	(M为临时矩阵，对应状态向量更新中的G)
				matrix M_next_covariance = matrix_multiply(&M, &next_covariance);
				matrix M_next_covariance_M_T = matrix_multiply(&M_next_covariance, &M_T);
				matrix_substitude(&next_covariance, 1, next_covariance.row, 
					1, next_covariance.row, &M_next_covariance_M_T);
				/*printf("next_covariance： \n");
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
				//printf("是已发现的landmark ! \n");
				
				//数据集中提到的H 观测对状态向量的更新
				matrix H = matrix_init(2, next_state_vector.row);
				calculate_H(&H, &next_state_vector, &z, *(*(map + landmark_information[0]) + 4));
				/*printf("H： \n");
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
				/*printf("S： \n");
				matrix_print(&S);
				printf("\n");*/

				matrix S_inv = matrix_inverse(&S);
				/*printf("S_inv： \n");
				matrix_print(&S_inv);
				printf("\n");*/

				//计算卡尔曼增益
				matrix next_covariance_H_T = matrix_multiply(&next_covariance, &H_T);
				matrix K = matrix_multiply(&next_covariance_H_T, &S_inv);
				/*printf("K： \n");
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
				/*printf("next_covariance： \n");
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

		//结束计时
		end = (float)clock();
		duration = duration + (end - begin);
		//打印消耗时间，单位：毫秒
		/*printf("累计消耗时间： \n");
		printf("%.4f毫秒", duration);
		printf("\n");*/

		//
		fprintf(fpWrite2, "%f %f\n", next_state_vector.data[0][0], next_state_vector.data[1][0]);
		fprintf(fpWrite3, "%d %d %d %d %f\n", i, next_state_vector.row, (next_state_vector.row - 3) / 2, next_state_vector.row * next_state_vector.row * 4 + next_state_vector.row * 4 + 16, duration);
		
		//
		
		matrix_free(&next_covariance);

		
	}

	//计时结束
	finish = (float)clock();

	//打印消耗时间，单位：毫秒
	printf("消耗时间： \n");
	printf("%.4f毫秒", (finish - start));
	printf("\n");

	//联合状态向量维数
	printf("联合状态向量维数：%d \n", next_state_vector.row);

	printf("next_state_vector： \n");
	matrix_print(&next_state_vector);
	printf("\n");

	/*printf("covariance： \n");
	matrix_print(&covariance);
	printf("\n");*/

	//释放地图内存
	for (i = 0; i < map_row; i++)
		free(map[i]);
	free(map);

	//释放里程计内存
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