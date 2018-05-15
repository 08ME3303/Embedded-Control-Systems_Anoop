#include "your_code.h"
#include "FreeRTOS.h"

/***
 *
 * This file is where you should add you tasks. You already know the structure
 * Required to do so from the work with the simulator.
 *
 * The function yourCodeInit() is set to automatically execute when the
 * quadrotor is started. This is where you need to create your tasks. The
 * scheduler that runs the tasks is already up and running so you should
 * NOT make a call to vTaskStartScheduler();.
 *
 * Below that you can find a few examples of useful function calls and code snippets.
 *
 * For further reference on how this is done. Look into the file stabilizer.c
 * which is usually handles the control of the crazyflie.
 *
 ***/
 #define SAMPLE_TIME 2
 #define GAMMA 0.98
 #define THRUST_CONVERT 445370
 
 typedef struct{
 	float roll;
 	float pitch;
 	} complementaryAngle;
 	
 static complementaryAngle comp_angle; //object for result of complementary filter
 static complementaryAngle acc_angle; // object for accelerometer angle from accelerometer sensor value
 static complementaryAngle angle_out;   // object to update the complementary filter angles
 
 static float K[4][4] = {{-0.2883,-0.3406,-0.0297,-0.0353},{-0.2883,-0.3406,-0.0297,-0.0353},{-0.2883,-0.3406,-0.0297,-0.0353},{-0.2883,-0.3406,-0.0297,-0.0353}};
 static float Kr[4][2] ={{-0.2883,-0.3406},{-0.2883,-0.3406},{-0.2883,-0.3406},{-0.2883,-0.3406}};
 static float krr[4] = {0,0,0,0};
 static float kx[4]= {0,0,0,0};
 static float controlInput[4] = {0,0,0,0};
<<<<<<< HEAD
 static float stateVector[4] = {0,0,0,0};
 
=======
>>>>>>> 06ef38a1b07fba1a1e2086d31be2060e24a15e86
 
 sensorData_t sensorData;
 setpoint_t setpoint;
 
 SemaphoreHandle_t xSemaphoreAngle;
 SemaphoreHandle_t xSemaphoreSensor;
 SemaphoreHandle_t xSemaphoreSet;
 
 
 void angle_init(complementaryAngle *arg)
 {
 	arg->roll = 0;
 	arg->pitch = 0;
 }
 
 void compFilter(void *arg)  // function to extract angle from acc data
 {
 	TickType_t xLastWakeTimeComp;
 	xLastWakeTimeComp = xTaskGetTickCount ();
 	
 	while(1)
 	{
<<<<<<< HEAD
 		vTaskDelayUntil(&xLastWakeTimeComp, M2T(SAMPLE_TIME));
=======
 		vTaskDelayUntil(&xLastWakeTimeAcc, M2T(SAMPLE_TIME));
>>>>>>> 06ef38a1b07fba1a1e2086d31be2060e24a15e86
 	
	    float f_x, f_y, f_z;
	    float gy_roll, gy_pitch;
	    
	    xSemaphoreTake(xSemaphoreSensor, portMAX_DELAY); // protecting sensor data
 		sensorAcquire(&sensorData);
 	
 		f_x = sensorData.acc.x;
	 	f_y = sensorData.acc.y;
	 	f_z = sensorData.acc.z;	
	 	gy_roll = sensorData.gyro.x;
 		gy_pitch = sensorData.gyro.y;
<<<<<<< HEAD
 		stateVector[2] = gy_roll;
 		stateVector[3] = gy_pitch;
=======
>>>>>>> 06ef38a1b07fba1a1e2086d31be2060e24a15e86
 		xSemaphoreGive(xSemaphoreSensor);                // releasing sensor
 		
 		acc_angle.roll = (atan2((-f_x),(sqrt(f_y*f_y + f_z*f_z))) * (180/M_PI));
 		acc_angle.pitch = (atan2(f_y,f_z) * (180/M_PI)); 	
 		
 		xSemaphoreTake(xSemaphoreAngle, portMAX_DELAY);  // protecting angle data
 		angle_out.roll = ((acc_angle.roll)*(1-GAMMA)) + (((gy_roll * SAMPLE_TIME) + angle_out.roll)*GAMMA);
 		angle_out.pitch = ((acc_angle.pitch)*(1-GAMMA)) + (((gy_pitch * SAMPLE_TIME) + angle_out.pitch)*GAMMA);	
<<<<<<< HEAD
		stateVector[0] = angle_out.roll;
		stateVector[1] = angle_out.pitch; 		
		xSemaphoreGive(xSemaphoreAngle); 	             // releasing angle
 		
=======
 		xSemaphoreGive(xSemaphoreAngle); 	             // releasing angle
>>>>>>> 06ef38a1b07fba1a1e2086d31be2060e24a15e86
 		}
 } 
 
 void setPointGen(void *arg3)
 {
 	TickType_t xLastWakeTimeSet;
<<<<<<< HEAD
 	xLastWakeTimeSet = xTaskGetTickCount ();
=======
>>>>>>> 06ef38a1b07fba1a1e2086d31be2060e24a15e86
 	while(1)
 	{
 		vTaskDelayUntil(&xLastWakeTimeSet, M2T(SAMPLE_TIME));
 		
 		//Generate Kr * r vector
 		
 		xSemaphoreTake(xSemaphoreSet, portMAX_DELAY);
 		commanderGetSetpoint(&setpoint);	
 		float ref_vec[2] = {setpoint.attitude.roll, setpoint.attitude.pitch};
 		
 		
 		for (i =0; i<4; i++)
 		{
 			for(j=0; j<2; j++)
 			{
 				krr[i] = krr[i] + ref_vec[j]*Kr[i][j];
 			}
 		}
<<<<<<< HEAD
 		xSemaphoreGive(xSemaphoreSet);
=======
 		xSemaphoreGive(xSemaphoreSet)
>>>>>>> 06ef38a1b07fba1a1e2086d31be2060e24a15e86
 	}
 }
 
 void LQR(void *arg2)
 {
 	uint16_t motorSignals[4];
 	int i, j, k;
 	TickType_t xLastWakeTimeLQR;
 	
 	while(1)
 	{
 		vTaskDelayUntil(&xLastWakeTimeLQR, M2T(SAMPLE_TIME));
 		//Generate K*x vector
<<<<<<< HEAD

=======
 		
 		xSemaphoreTake(xSemaphoreAngle, portMAX_DELAY);
 		xSemaphoreTake(xSemaphoreSensor, portMAX_DELAY);
 		float state_vec[4] = {angle_out.roll, angle_out.pitch, sensorData.gyro.x, sensorData.gyro.y};
 		xSemaphoreGive(xSemaphoreAngle);
 		xSemaphoreGive(xSemaphoreSensor);
>>>>>>> 06ef38a1b07fba1a1e2086d31be2060e24a15e86
 		
 		for(i= 0; i<4; i++)
 		{
 			for (j = 0; i <4; j++)
 			{
 				kx[i] = kx[i] + stateVector[j]*K[i][j];
 			}
 		}
 		
 		//Generate Control signals
 		
 		for(k=0; k<4; k++)
 		{
 			controlInput[i] = (krr[i] - kx[i])*THRUST_CONVERT;
 		}
 		
 		xSemaphoreTake(xSemaphoreSet, portMAX_DELAY);
        // Set motor signals (type UINT16_T)
        motorsSetRatio(MOTOR_M1, setpoint.thrust + controlInput[0]);
        motorsSetRatio(MOTOR_M2, setpoint.thrust + controlInput[1]);
        motorsSetRatio(MOTOR_M3, setpoint.thrust + controlInput[2]);
        motorsSetRatio(MOTOR_M4, setpoint.thrust + controlInput[3]);
		xSemaphoreGive(xSemaphoreSet);
 	}
 }


 void yourCodeInit(void)
 {
 	taskHandle_t taskComp;
 	taskHandle_t taskContr;
 	taskHandle_t taskSet;
 	
 	sensorInit();
 	
 	angle_init(&comp_angle);    //initialize angles from complementary filter
 	angle_init(&acc_angle);     //initialize angles from accelerometer data
 	
 	xSemaphoreAngle = xSemaphoreCreateBinary();
 	xSemaphoreGive(xSemaphoreAngle);
 	xSemaphoreSet = xSemaphoreCreateBinary();
 	xSemaphoreGive(xSemaphoreSet);
 	xSemaphoreSensor = xSemaphoreCreateBinary();
 	xSemaphoreGive(xSemaphoreSensor);
   
<<<<<<< HEAD
    xTaskCreate(compFilter, "Complementary Filter", configMINIMAL_STACK_SIZE, NULL, 3, &taskComp);
    xTaskCreate(LQR, "Controller", configMINIMAL_STACK_SIZE, NULL, 2, &taskContr);
    xTaskCreate(setPointGen, "Set Point Generator", configMINIMAL_STACK_SIZE, NULL, 1, &taskSet);
=======
    xTaskCreate(compFilter, "Complementary Filter", configMINIMAL_STACK_SIZE, NULL, 3, taskComp);
    xTaskCreate(LQR, "Controller", configMINIMAL_STACK_SIZE, NULL, 2, taskContr);
    xTaskCreate(setPointGen, "Set Point Generator", configMINIMAL_STACK_SIZE, NULL, 1 taskSet);
>>>>>>> 06ef38a1b07fba1a1e2086d31be2060e24a15e86
 }



/*************************************************
 * WAIT FOR SENSORS TO BE CALIBRATED
 ************************************************/
// lastWakeTime = xTaskGetTickCount ();
// while(!sensorsAreCalibrated()) {
//     vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
// }



/*************************************************
 * RETRIEVE THE MOST RECENT SENSOR DATA
 *
 * The code creates a variable called sensorData and then calls a function
 * that fills this variable with the latest data from the sensors.
 *
 * sensorData_t sensorData = struct {
 *     Axis3f acc;
 *     Axis3f gyro;
 *     Axis3f mag;
 *     baro_t baro;
 *     zDistance_t zrange;
 *     point_t position;
 * }
 *
 ************************************************/
// sensorData_t sensorData;
// sensorsAcquire(&sensorData);



/*************************************************
 * RETRIEVE THE SET POINT FROM ANY EXTERNAL COMMAND INTERFACE
 *
 * The code creates a variable called setpoint and then calls a function
 * that fills this variable with the latest command input.
 *
 * setpoint_t setpoint = struct {
 *     uint32_t timestamp;
 *
 *     attitude_t attitude;      // deg
 *     attitude_t attitudeRate;  // deg/s
 *     quaternion_t attitudeQuaternion;
 *     float thrust;
 *     point_t position;         // m
 *     velocity_t velocity;      // m/s
 *     acc_t acceleration;       // m/s^2
 *     bool velocity_body;       // true if velocity is given in body frame; false if velocity is given in world frame
 *
 *     struct {
 *         stab_mode_t x;
 *         stab_mode_t y;
 *         stab_mode_t z;
 *         stab_mode_t roll;
 *         stab_mode_t pitch;
 *         stab_mode_t yaw;
 *         stab_mode_t quat;
 *     } mode;
 * }
 *
 ************************************************/
// setpoint_t setpoint;
// commanderGetSetpoint(&setpoint);



/*************************************************
 * SENDING OUTPUT TO THE MOTORS
 *
 * The code sends an output to each motor. The output should have the be
 * of the typ unsigned 16-bit integer, i.e. use variables such as:
 * uint16_t value_i
 *
 ************************************************/
// motorsSetRatio(MOTOR_M1, value_1);
// motorsSetRatio(MOTOR_M2, value_2);
// motorsSetRatio(MOTOR_M3, value_3);
// motorsSetRatio(MOTOR_M4, value_4);


/*************************************************
 * LOGGING VALUES THAT CAN BE PLOTTEN IN PYTHON CLIENT
 *
 * We have already set up three log blocks to for the accelerometer data, the
 * gyro data and the setpoints, just uncomment the block to start logging. Use
 * them as reference if you want to add custom blocks.
 *
 ************************************************/

/*
LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)
*/

/*
LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyro.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyro.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)
*/

/*
LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, roll, &setpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)
*/
