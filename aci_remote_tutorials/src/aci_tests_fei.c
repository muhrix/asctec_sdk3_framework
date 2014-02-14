/*
 * aci_tests_fei.c
 *
 *  Created on: 11 Feb 2014
 *      Author: Murilo F. M.
 *      Email:  muhrix@gmail.com
 *
 */

#include "aci_remote_v100/asctecCommIntf.h"

#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h>  // File control definitions
#include <termios.h> // POSIX terminal control definitionss
#include <pthread.h>
#include <time.h>

int fd;

unsigned char ctrl_mode = 0;
unsigned char ctrl_enabled = 0;
unsigned char disable_motor_onoff_by_stick = 0;

unsigned char motor1 = 0;
unsigned char motor2 = 0;
unsigned char motor3 = 0;
unsigned char motor4 = 0;

unsigned char cmd_ready = 0;
unsigned char motor_start = 1;

//u_int16_t do_beep = 0;
unsigned short do_beep = 0;
unsigned short my_test = 0;

int32_t angle_pitch;
int32_t angle_roll;
int32_t angle_yaw;

int16_t motor_rpm_0;
int16_t motor_rpm_1;
int16_t motor_rpm_2;
int16_t motor_rpm_3;

u_int16_t wp_nav_status;
u_int16_t wp_dist_wp;


unsigned char var_received;

void transmit(void* byte, unsigned short cnt);
void varListUpdateFinished(void);
void cmdListUpdateFinished(void);
void paramListUpdateFinished(void);
void *aciThread(void);
void startStopMotor(int);

int main(int argc, char *argv[]) {

	printf("sizeof(unsigned char): %d\n", sizeof(unsigned char));
	printf("sizeof(char): %d\n", sizeof(char));
	printf("sizeof(u_int16_t): %d\n", sizeof(u_int16_t));
	printf("sizeof(unsigned short): %d\n", sizeof(unsigned short));
	printf("sizeof(short): %d\n", sizeof(short));
	printf("sizeof(unsigned int): %d\n", sizeof(unsigned int));
	printf("sizeof(int): %d\n", sizeof(int));

	var_received = 0;
	angle_pitch = 0;
	angle_roll = 0;
	angle_yaw = 0;

	motor_rpm_0 = 0;
	motor_rpm_1 = 0;
	motor_rpm_2 = 0;
	motor_rpm_3 = 0;

	pthread_t p_acithread;

	fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	struct termios port_settings; // structure to store the port settings in

	cfsetispeed(&port_settings, B230400);    // set baud rates
	port_settings.c_cflag = B230400 | CS8 | CREAD | CLOCAL;
	port_settings.c_iflag = IGNPAR;
	port_settings.c_oflag = 0;
	port_settings.c_lflag = 0;
	tcsetattr(fd, TCSANOW, &port_settings); // apply the settings to the port

	aciInit();
	aciSetSendDataCallback(&transmit);
	aciSetVarListUpdateFinishedCallback(&varListUpdateFinished);
	aciSetCmdListUpdateFinishedCallback(&cmdListUpdateFinished);
	aciSetParamListUpdateFinishedCallback(&paramListUpdateFinished);
	aciSetEngineRate(100, 10);

	pthread_create( &p_acithread, NULL, aciThread, NULL);
	aciGetDeviceCommandsList();
	aciGetDeviceParametersList();
	aciGetDeviceVariablesList();

	/*
	printf("Waiting for command list...\n");
	while(!cmd_ready) usleep(1000);
	while(motor_start) {
		printf("Start Motor number (0 to quit or 9 to beep): ");
		scanf("%i",&motor_start);

		if (motor_start == 9) {
			doBeep = 1;
			//aciUpdateCmdPacket(2);
			aciUpdateCmdPacket(0);
			doBeep = 0;
		}
		else
			startStopMotor(motor_start);
	}
	*/

	pthread_join(p_acithread, NULL);
}

void startStopMotor(int motor) {
	switch(motor) {
	case 1:
		motor1 = (motor1>0)? 0 : 10;
		break;
	case 2:
		motor2 = (motor2>0)? 0 : 10;
		break;
	case 3:
		motor3 = (motor3>0)? 0 : 10;
		break;
	case 4:
		motor4 = (motor4>0)? 0 : 10;
		break;
	}
	aciUpdateCmdPacket(0);
}

void transmit(void* byte, unsigned short cnt) {
	unsigned char *tbyte = (unsigned char *)byte;
	for (int i = 0; i < cnt; i++) {
		write(fd, &tbyte[i], 1);
	}
}

void *aciThread(void) {
	int timecounter=0;
	int result = 0;
	unsigned char data = 0;
	while(1) {
		timecounter++;
		result = read(fd, &data, 1);
		while (result > 0) {
			aciReceiveHandler(data);
			result = read(fd, &data, 1);
		}
		if(timecounter>100) {
			if(var_received && cmd_ready) {
				//printf("Angles before Synchronising: %f\t%f\t%f\n", (double)(angle_pitch)/1000, (double)(angle_roll)/1000 , (double)(angle_yaw)/1000);
				aciSynchronizeVars();
				//printf("Angles after Synchronising: %f\t%f\t%f\n---------\n", (double)(angle_pitch)/1000, (double)(angle_roll)/1000 , (double)(angle_yaw)/1000);
				printf("Motor RPM: %d\t%d\t%d\t%d\n", motor_rpm_0, motor_rpm_1, motor_rpm_2, motor_rpm_3);

				//printf("Waypoint navigation status: %d\n", wp_nav_status);
				//printf("Waypoint distance to waypoint: %d\n", wp_dist_wp);
				printf("do_beep: %d\n", do_beep);
				printf("my_test received  : %d\n", my_test);
				printf("my_test calculated: %d\n", (do_beep-5 + do_beep + 1));

				do_beep = my_test + 5;
				//do_beep = 1;
				aciUpdateCmdPacket(0);
			}
			else {
				printf("still waiting for lists...\n");
			}
			timecounter = 0;
		}
		aciEngine();
		usleep(10000);
	}
	return NULL;
}

void varListUpdateFinished(void) {
	printf("variables list updated!\n");
	aciAddContentToVarPacket(0,0x0300,&angle_pitch);
	aciAddContentToVarPacket(0,0x0301,&angle_roll);
	aciAddContentToVarPacket(0,0x0302,&angle_yaw);
	aciAddContentToVarPacket(0,0x0100,&motor_rpm_0);
	aciAddContentToVarPacket(0,0x0101,&motor_rpm_1);
	aciAddContentToVarPacket(0,0x0102,&motor_rpm_2);
	aciAddContentToVarPacket(0,0x0103,&motor_rpm_3);
	aciAddContentToVarPacket(0,0x1012,&wp_nav_status);
	aciAddContentToVarPacket(0,0x1013,&wp_dist_wp);
	aciAddContentToVarPacket(0,0x1015,&my_test);
	aciSetVarPacketTransmissionRate(0,10);
	aciVarPacketUpdateTransmissionRates();
	aciSendVariablePacketConfiguration(0);
	var_received = 1;
}

void cmdListUpdateFinished() {
	printf("command list updated!\n");

	aciAddContentToCmdPacket(0, 0x0500, &motor1);
	aciAddContentToCmdPacket(0, 0x0501, &motor2);
	aciAddContentToCmdPacket(0, 0x0502, &motor3);
	aciAddContentToCmdPacket(0, 0x0503, &motor4);
	aciAddContentToCmdPacket(0, 0x1014, &do_beep);
	aciAddContentToCmdPacket(1, 0x0600, &ctrl_mode);
	aciAddContentToCmdPacket(1, 0x0601, &ctrl_enabled);
	aciAddContentToCmdPacket(1, 0x0602, &disable_motor_onoff_by_stick);
	aciSendCommandPacketConfiguration(0, 0);
	aciSendCommandPacketConfiguration(1, 1);
	motor1 = 0;
	motor2 = 0;
	motor3 = 0;
	motor4 = 0;
	do_beep = 0;
	ctrl_mode = 0;
	ctrl_enabled = 1;
	disable_motor_onoff_by_stick = 1;
	aciUpdateCmdPacket(0);
	aciUpdateCmdPacket(1);
	cmd_ready = 1;
}

void paramListUpdateFinished() {

}


