// #include "youbot/YouBotBase.hpp"
// #include "youbot/YouBotManipulator.hpp"

#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
//#include <io.h>
#include <fstream>
#include <string>
#include <sstream>

#include "youbot_driver/youbot/YouBotBase.hpp"

#include <youbot_camera/RT_youbot_base.h>

using namespace youbot;
using namespace youbot_camera_suite;

#define YOUBOT_CONFIGURATIONS_DIR "/home/sees/CommLineProg/nonproProg/catkin_ws/src/kuka-youbot-driver/config/"

int main(int argc, char **argv)
{

	ros::init(argc, argv, "talker");
	/* define velocities */
	double translationalVelocity = 0.05; //meter_per_second
	double rotationalVelocity = 0.2; //radian_per_second

	try 
	{
		RT_youbot_base robot("myRobot", YOUBOT_CONFIGURATIONS_DIR);
		bool youBotHasBase = true;
	} 
	catch (std::exception& e)
	{
		LOG(warning) << e.what();
		youBotHasBase = false;
	}

	ros::Rate loop_rate(10);

	while (ros::ok())
  {

    std_msgs::String msg;
  
  	std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
   
    ROS_INFO("%s", msg.data.c_str());
   

    chatter_pub.publish(msg);
  
    ros::spinOnce();

    loop_rate.sleep();
	}

	
	// Углы, соответствующие начальному положению схвата
	double A10 = 2.9496;  
    double A20 =0.2692270632844107;
    double A30 = -2.2057268813334305;
    double A40 =3.1053961448439167;
	double A50 = 2.93;	

	// Углы, получаемые решением обратной задачи кинематики
    double phi10, phi20, phi30, phi40, phi50, phi2, phi3, phi4, dphi21, dphi31, phi21, phi31;

	// Измеренные углы
    double  A1q, A2q, A3q, A4q, A5q;

	// Командные значения углов
	double A1, A2, A3, A4, A5;

	// Элементы матрицы Якоби
	double J22,  J23,  J32,  J33;
	double detJ;

	// Желаемые скорости
    double vAx1, vAz1, x1A, z1A, X1Ad, Z1Ad, Ex, Ez;

	// Вспомогательные пременные для метода Эйлера
	double K20, K30;

	// Время // НАДО ПОМЕНЯТЬ TMAX А ТО ЭЙЛЕР С ОБРАТНОЙ СВЯЗЬЮ ЗАПЛАЧЕТ!
	double t0 = 0, dt = 0.05, tmax = 10.0, DT, t1;

	// Размеры звеньев
	double d1=0.033, l1=0.075, l2=0.155, l3=0.135, l4=0.081, l5=0.137, l45;
	l45 = l4 + l5;

	//Омега
	double om = 2 * 3.14 / tmax;

	// Параметры программного движения
	//////////////КОЭФФИЦИЕНТЫ ОБРАТНОЙ СВЯЗИ???????????????///////
	double c11 = 0.355, c12 = 0.005, c21 = 0.125, c22 = -0.175;
	double cv1 = c12*om;
	double cv2 = c22*om;

	// Коэффициент обратной связи kp = 4...6
	double kp=6.0;

	// Координаты схвата, ошибки позиционирования
	double x0, z0, xq, zq, ex, ez;


	// Файл
    FILE *tel;
    FILE *param;

	//
	// Предварительные вычисления
	//
	// Длительность такта в мс
	DT = (1000 * dt);

  while (true)
  {
     cm.update(robot.get_time(), robot.get_period());
     sleep();
  }

// 	try
// 	{
// 		/*
// 		 * Simple sequence of commands to the youBot:
// 		 */
// //
// //     Платформа неподвижна
// //
// 		if (youBotHasBase)
// 		{
// 			/* stop base */
// 			longitudinalVelocity = 0 * meter_per_second;
// 			transversalVelocity = 0 * meter_per_second;
// 			angularVelocity = 0 * radian_per_second;
// 			myYouBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
// 			LOG(info) << "stop base";
// 		}
// //
// //     Задействуем только манипулятор
// //
// 		if (youBotHasArm)
// 		{
// 			//////////////////////////////
// 			// Метод Эйлера///////////////
// 			//////////////////////////////
// 			//////////////////////////////////////////////////////////////////////////
// 			// Открываем файл для записи показаний с датчиков
//             tel = fopen("C-12A-17_5_SPEED.txt", "w");
//             param = fopen("speed.txt","r");
// 			// Управляющие сигналы на приводы
// 			A1 = A10;
// 			A2 = A20;
// 			A3 = A30;
// 			A4 = A40;
// 			A5 = A50;

// 			// Выставляем манипулятор в исходное положение
// 			desiredJointAngle.angle = A1 * radian;
// 			myYouBotManipulator->getArmJoint(1).setData(desiredJointAngle);
// 			desiredJointAngle.angle = A2 * radian;
// 			myYouBotManipulator->getArmJoint(2).setData(desiredJointAngle);
// 			desiredJointAngle.angle = A3 * radian;
// 			myYouBotManipulator->getArmJoint(3).setData(desiredJointAngle);
// 			desiredJointAngle.angle = A4 * radian;
// 			myYouBotManipulator->getArmJoint(4).setData(desiredJointAngle);
// 			desiredJointAngle.angle = A5 * radian;
// 			myYouBotManipulator->getArmJoint(5).setData(desiredJointAngle);
// 			LOG(info) << "fold arm";
// 			int i = 0;
//              for (i=0;i<=60;i++){
                 
//              //      A3, &A4
//            // while((fscanf(param, "%f %f %f \n", &A2, &A3, &A4)!=EOF)&&(i<1000)){
// 				//i++;
// 				// Измеряем углы в сочленениях, приводим значения к выбранной кинематической схеме
// 				myYouBotManipulator->getArmJoint(1).getData(sensedAngle);
// 				A1q =(sensedAngle.angle)/radian;
// 				myYouBotManipulator->getArmJoint(2).getData(sensedAngle);
// 				A2q =(sensedAngle.angle)/radian;
// 				myYouBotManipulator->getArmJoint(3).getData(sensedAngle);
// 				A3q = (sensedAngle.angle)/radian;
// 				myYouBotManipulator->getArmJoint(4).getData(sensedAngle);
// 				A4q = (sensedAngle.angle)/radian;
// 				myYouBotManipulator->getArmJoint(5).getData(sensedAngle);
// 				A5q =(sensedAngle.angle)/radian;
// 				// Записываем измерения в файл
// 				fprintf(tel,"%7.4f %7.4f %7.4f %7.4f %7.4f \n", A1q, A2q, A3q, A4q, A5q);
// 				SLEEP_MILLISEC(50);
// 				}
// 				/*

// 			SLEEP_MILLISEC(3000);

// 			//for (t0 = 0; t0 <= tmax; t0 = t0 + dt)

// 			int i = 0;
//              while((!feof(param))){
//                    float tempA2,tempA3,tempA4;
//                    fscanf(param, "%lf %lf %lf \n", &A2,&A3,&A4);
//              //      A3, &A4
//            // while((fscanf(param, "%f %f %f \n", &A2, &A3, &A4)!=EOF)&&(i<1000)){
// 				i++;
// 				// Измеряем углы в сочленениях, приводим значения к выбранной кинематической схеме
// 				myYouBotManipulator->getArmJoint(2).getData(sensedAngle);
// 				A2q =(sensedAngle.angle)/radian;
// 				myYouBotManipulator->getArmJoint(3).getData(sensedAngle);
// 				A3q = (sensedAngle.angle)/radian;
// 				myYouBotManipulator->getArmJoint(4).getData(sensedAngle);
// 				A4q = (sensedAngle.angle)/radian;
// 				// Записываем измерения в файл
// 				fprintf(tel,"%7.4f %7.4f %7.4f %7.4f %7.4f %7.4f\n", A20, A2q, A30, A3q, A40, A4q);
// 				/*
// 				// Желаемые компонеты скорости схвата
// 				vAx1 = -cv1*sin(om*t0);
// 				vAz1 = -cv2*sin(om*t0);

// 				// Расчёт матрицы Якоби:
// 				J23= l3*cos(phi20 + phi30) + l45*cos(phi20 + phi30 + phi40);
// 				J22 = J23 + l2*cos(phi20);
// 				J33= -l3*sin(phi20 + phi30) - l45*sin(phi20 + phi30 + phi40);
// 				J32 = J33 - l2*sin(phi20);

// 				detJ = J22*J33 - J23*J32;

// 				// Коэффициенты для метода эйлера
// 				K20 = (J33*vAx1 - J23*vAz1) / detJ;
// 				K30 = (-J32*vAx1 + J22*vAz1) / detJ;

// 				phi2 = phi20 + dt*K20;
// 				phi3 = phi30 + dt*K30;
// 				// Управляющие сигналы A2 и A3 на приводы
// 				A2 = phi2 + 1.1345;
// 				A3 = phi3 - 2.5654;
// 				*/
// 				// Проверка величин углов на попадание в допустимые диапазоны
// 			// 	if((A2>0.01)&&(A2<2.61799)&&(A3>-4.8)&&(A3<-0.01)&&(A4>0.022)&&(A4<3.4292))
// 			// 	{
// 			// 		// Отправляем команды на приводы активных сочленений
// 			// 		desiredJointAngle.angle = A2 * radian;
// 			// 		myYouBotManipulator->getArmJoint(2).setData(desiredJointAngle);
// 			// 		desiredJointAngle.angle = A3 * radian;
// 			// 		myYouBotManipulator->getArmJoint(3).setData(desiredJointAngle);
// 			// 		desiredJointAngle.angle = A4 * radian;
// 			// 		myYouBotManipulator->getArmJoint(4).setData(desiredJointAngle);
// 			// 		// Запоминаем значения углов для следующего такта
// 			// 		A20=A2;
// 			// 		A30=A3;
// 			// 		A40=A4;
// 			// 	}
// 			// 	else
// 			// 	{
// 			// 		LOG(info) << "Angles are wrong: " << A2 << "  " << A3 << "  " << A4 << "\n";
// 			// 		LOG(info) << "Failed step: " << i;
// 			// 	}
// 			// 	SLEEP_MILLISEC(DT);

//    //          }
// 			// if (i>=1000) 
// 			// {
// 			// 	LOG(info) << "Max step: " << i;
// 			// }
// 			// // Закрываем файл
//             fclose(tel);
//           //  SLEEP_MILLISEC(3000);
// 			////////////////////////////////////////////////////////////////////////////////
//             /*
// 			////////////////////////////////////////////////
// 			// Метод Эйлера с обратной связью///////////////
// 			////////////////////////////////////////////////
// 			///////////////////////////////////////////////////////////////////////////////
// 			// Открываем файл для записи показаний с датчиков
// 			tel = fopen("EuFB_15.txt", "w");

// 			// Углы, соответствующие начальному положению схвата
// 			phi10 = phi1n;
// 			phi20 = phi2n;
// 			phi30 = phi3n;
// 			phi40 = phi4n;
// 			phi50 = phi5n;
// 			// Управляющие сигналы на приводы
// 			A1 = 2.9496 - phi10;
// 			A2 = phi20 + 1.1345;
// 			A3 = phi30 - 2.5654;
// 			A4 = phi40 + 1.8290;
// 			A5 = 2.9300 - phi50;

// 			// Выставляем манипулятор в исходное положение
// 			desiredJointAngle.angle = A1 * radian;
// 			myYouBotManipulator->getArmJoint(1).setData(desiredJointAngle);
// 			desiredJointAngle.angle = A2 * radian;
// 			myYouBotManipulator->getArmJoint(2).setData(desiredJointAngle);
// 			desiredJointAngle.angle = A3 * radian;
// 			myYouBotManipulator->getArmJoint(3).setData(desiredJointAngle);
// 			desiredJointAngle.angle = A4 * radian;
// 			myYouBotManipulator->getArmJoint(4).setData(desiredJointAngle);
// 			desiredJointAngle.angle = A5 * radian;
// 			myYouBotManipulator->getArmJoint(5).setData(desiredJointAngle);
// 			LOG(info) << "fold arm";
// 			SLEEP_MILLISEC(3000);

// 			for (t0 = 0; t0 <= tmax; t0 = t0 + dt)
// 			{
// 				// Измеряем углы в сочленениях, приводим значения к выбранной кинематической схеме
// 				myYouBotManipulator->getArmJoint(2).getData(sensedAngle);
// 				phi2q = -1.1345 + (sensedAngle.angle) / radian;
// 				myYouBotManipulator->getArmJoint(3).getData(sensedAngle);
// 				phi3q = 2.5654 + (sensedAngle.angle) / radian;
// 				// Записываем измерения в файл
// 				fprintf(tel, "%6.3f %7.4f %7.4f %7.4f %7.4f\n", t0, phi20, phi2q, phi30, phi3q);

// 				// Желаемые координаты схвата
// 				x0 = c11 + c12*cos(om*t0);
// 				z0 = c21 + c22*cos(om*t0);

// 				// Реальные координаты схвата
// 				xq = d1 + l2*sin(phi2q) + l3*sin(phi2q + phi3q) + l45*sin(phi2q + phi3q + phi40);
// 				zq = l1 + l2*cos(phi2q) + l3*cos(phi2q + phi3q) + l45*cos(phi2q + phi3q + phi40);

// 				//Ошбики позиционирования
// 				ex = x0 - xq;
// 				ez = z0 - zq;

// 				// Желаемые компонеты скорости схвата
// 				vAx1 = -cv1*sin(om*t0) + kp*ex;
// 				vAz1 = -cv2*sin(om*t0) + kp*ez;

// 				// Расчёт матрицы Якоби:
// 				J23 = l3*cos(phi20 + phi30) + l45*cos(phi20 + phi30 + phi40);
// 				J22 = J23 + l2*cos(phi20);
// 				J33 = -l3*sin(phi20 + phi30) - l45*sin(phi20 + phi30 + phi40);
// 				J32 = J33 - l2*sin(phi20);

// 				detJ = J22*J33 - J23*J32;

// 				// Коэффициенты для метода эйлера
// 				K20 = (J33*vAx1 - J23*vAz1) / detJ;
// 				K30 = (-J32*vAx1 + J22*vAz1) / detJ;

// 				phi2 = phi20 + dt*K20;
// 				phi3 = phi30 + dt*K30;
// 				// Управляющие сигналы A2 и A3 на приводы
// 				A2 = phi2 + 1.1345;
// 				A3 = phi3 - 2.5654;
// 				// Проверка величин углов на попадание в допустимые диапазоны
// 				if ((A2>0.01) && (A2<2.61799) && (A3>-4.8) && (A3<-0.01))
// 				{
// 					// Отправляем команды на приводы активных сочленений
// 					desiredJointAngle.angle = A2 * radian;
// 					myYouBotManipulator->getArmJoint(2).setData(desiredJointAngle);
// 					desiredJointAngle.angle = A3 * radian;
// 					myYouBotManipulator->getArmJoint(3).setData(desiredJointAngle);
// 					// Запоминаем значения углов для следующего такта
// 					phi20 = phi2;
// 					phi30 = phi3;
// 				}
// 				else
// 				{
// 					LOG(info) << "Eiler s obr svaziu, nedopustimie ugli";
// 					break;
// 				}
// 				SLEEP_MILLISEC(DT);
// 			}
// 			// Закрываем файл
// 			fclose(tel);
// 			SLEEP_MILLISEC(3000);
// 			////////////////////////////////////////////////////////////////////////////////

// 			/////////////////////////////
// 			// Метод Ньютона///////////////
// 			/////////////////////////////
// 			///////////////////////////////////////////////////////////////////////////////
// 			// Открываем файл для записи показаний с датчиков
// 			tel = fopen("Ord2_15.txt", "w");

// 			// Углы, соответствующие начальному положению схвата
// 			phi10 = phi1n;
// 			phi20 = phi2n;
// 			phi30 = phi3n;
// 			phi40 = phi4n;
// 			phi50 = phi5n;
// 			// Управляющие сигналы на приводы
// 			A1 = 2.9496 - phi10;
// 			A2 = phi20 + 1.1345;
// 			A3 = phi30 - 2.5654;
// 			A4 = phi40 + 1.8290;
// 			A5 = 2.9300 - phi50;

//             // Выставляем манипулятор в исходное положение
// 			desiredJointAngle.angle = A1 * radian;
// 			myYouBotManipulator->getArmJoint(1).setData(desiredJointAngle);
// 			desiredJointAngle.angle = A2 * radian;
// 			myYouBotManipulator->getArmJoint(2).setData(desiredJointAngle);
// 			desiredJointAngle.angle = A3 * radian;
// 			myYouBotManipulator->getArmJoint(3).setData(desiredJointAngle);
// 			desiredJointAngle.angle = A4 * radian;
// 			myYouBotManipulator->getArmJoint(4).setData(desiredJointAngle);
// 			desiredJointAngle.angle = A5 * radian;
// 			myYouBotManipulator->getArmJoint(5).setData(desiredJointAngle);
// 			LOG(info) << "fold arm";
// 			SLEEP_MILLISEC(3000);

// 			for (t0 = 0; t0 <= tmax; t0 = t0 + dt)
// 			{
// 				// Измеряем углы в сочленениях, приводим значения к выбранной кинематической схеме
// 				myYouBotManipulator->getArmJoint(2).getData(sensedAngle);
// 				phi2q = -1.1345 + (sensedAngle.angle) / radian;
// 				myYouBotManipulator->getArmJoint(3).getData(sensedAngle);
// 				phi3q = 2.5654 + (sensedAngle.angle) / radian;
// 				// Записываем измерения в файл
// 				fprintf(tel, "%6.3f %7.4f %7.4f %7.4f %7.4f\n", t0, phi20, phi2q, phi30, phi3q);

// 				////// Перый шаг по методу Ньютона//////
// 				// Желаемые компонеты скорости схвата
// 				x1A = d1 + l2*sin(phi20) + l3*sin(phi20 + phi30) + l45*sin(phi20 + phi30 + phi40);
//                 z1A = l1 + l2*cos(phi20) + l3*cos(phi20 + phi30) +  l45*cos(phi20 + phi30 + phi40);
//                 X1Ad = c11 + c12*cos(om*(t0 + dt));
//                 Z1Ad = c21 + c22*cos(om*(t0 + dt));

// 				Ex = x1A - X1Ad;
//                 Ez = z1A - Z1Ad;

//                 J23 = l3*cos(phi20 + phi30) + l45*cos(phi20 + phi30 + phi4);
//                 J22 = l2*cos(phi20) + J23;
//                 J33 = -l3*sin(phi20 + phi30) - l45*sin(phi20 + phi30 + phi4);
//                 J32 = -l2*sin(phi20) + J33;

//                 detJ = J22*J33 - J23*J32;
//                 dphi2 = (J33*Ex - J23*Ez)/detJ;
//                 dphi3 = (-J32*Ex + J22*Ez)/detJ;
//                 phi2 = phi20 - dphi2;
//                 phi3 = phi30 - dphi3;

//                 x1A = d1 + l2*sin(phi2) + l3*sin(phi2 + phi3) + l45*sin(phi2 + phi3 + phi4);
//                 z1A = l1 + l2*cos(phi2) + l3*cos(phi2 + phi3) + l45*cos(phi2 + phi3 + phi4);
//                 Ex = x1A - X1Ad;
//                 Ez = z1A - Z1Ad;
//                 J23 = l3*cos(phi2 + phi3) + l45*cos(phi2 + phi3 + phi4);
//                 J22 = l2*cos(phi2) + J23;
//                 J33 = -l3*sin(phi2 + phi3) - l45*Sin(phi2 + phi3 + phi4);
//                 J32 = -l2*sin(phi2) + J33;
//                 detJ = J22*J33 - J23*J32;
//                 dphi21 = (J33*Ex - J23*Ez)/detJ;
//                 dphi31 = (-J32*Ex + J22*Ez)/detJ;
//                 phi21 = phi2 - dphi21;
//                 phi31 = phi3 - dphi31;
//                           //  (* phi2 и phi3 - управляющие сигналы на приводы *)
//                 phi2 = phi21;
//                 phi3 = phi31;
// 				// Управляющие сигналы A2 и A3 на приводы
// 				A2 = phi2 + 1.1345;
// 				A3 = phi3 - 2.5654;
// 				// Проверка величин углов на попадание в допустимые диапазоны
// 				if ((A2>0.01) && (A2<2.61799) && (A3>-4.8) && (A3<-0.01))
// 				{
// 					// Отправляем команды на приводы активных сочленений
// 					desiredJointAngle.angle = A2 * radian;
// 					myYouBotManipulator->getArmJoint(2).setData(desiredJointAngle);
// 					desiredJointAngle.angle = A3 * radian;
// 					myYouBotManipulator->getArmJoint(3).setData(desiredJointAngle);
// 					// Запоминаем значения углов для следующего такта
// 					phi20 = phi2;
// 					phi30 = phi3;
// 				}
// 				else
// 				{
// 					LOG(info) << "Runge, nedopustimie ugli";
// 				}
// 				SLEEP_MILLISEC(DT);
// 			}
// 			// Закрываем файл
// 			fclose(tel);
// 			SLEEP_MILLISEC(3000);
// 			////////////////////////////////////////////////////////////////////////////////

// */
// 			// Возвращаем манипулятор в сложенное состояние
// 			desiredJointAngle.angle = 0.11 * radian;
// 			myYouBotManipulator->getArmJoint(1).setData(desiredJointAngle);
// 			desiredJointAngle.angle = 0.11 * radian;
// 			myYouBotManipulator->getArmJoint(2).setData(desiredJointAngle);
// 			desiredJointAngle.angle =-0.11 * radian;
// 			myYouBotManipulator->getArmJoint(3).setData(desiredJointAngle);
// 			desiredJointAngle.angle = 0.11 * radian;
// 			myYouBotManipulator->getArmJoint(4).setData(desiredJointAngle);
// 			desiredJointAngle.angle = 2.93 * radian;
// 			myYouBotManipulator->getArmJoint(5).setData(desiredJointAngle);
// 			LOG(info) << "unfold arm";
// 			SLEEP_MILLISEC(3000);
// 		}

// 	} catch (std::exception& e) {
// 		std::cout << e.what() << std::endl;
// 		std::cout << "unhandled exception" << std::endl;
// 	}
	
	/* clean up */
	if (myYouBotBase) {
		delete myYouBotBase;
		myYouBotBase = 0;
	}
	if (myYouBotManipulator) {
		delete myYouBotManipulator;
		myYouBotManipulator = 0;
	}

	LOG(info) << "Done.";

	return 0;
}

