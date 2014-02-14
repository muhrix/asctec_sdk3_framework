#include "aci_remote_v100/asctecCommIntf.h"

#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h>  // File control definitions
#include <termios.h> // POSIX terminal control definitions
#include <pthread.h>
#include <time.h>

int fd;

unsigned char ctrl_mode = 0;
unsigned char ctrl_enabled = 0;
unsigned char disable_motor_onoff_by_stick = 0;

unsigned char motor1=0;
unsigned char motor2=0;
unsigned char motor3=0;
unsigned char motor4=0;

unsigned char cmd_ready = 0;
unsigned char motor_start=1;

unsigned short doBeep = 0;

void transmit(void* byte, unsigned short cnt);
void varListUpdateFinished();
void cmdListUpdateFinished();
void paramListUpdateFinished();
void *aciThread(void);
void startStopMotor(int);

int main(int argc, char *argv[]) {

	pthread_t p_acithread;

	fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	struct termios port_settings; // structure to store the port settings in

	cfsetispeed(&port_settings, B230400); // set baud rates
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

	pthread_create(&p_acithread, NULL, aciThread, NULL);
	aciGetDeviceCommandsList();
	printf("Waiting for command list...\n");
	while(!cmd_ready) usleep(1000);
	while(motor_start) {
		printf("Start Motor number (0 to quit or 9 to beep): ");
		scanf("%i",&motor_start);

		if (motor_start == 9) {
			doBeep = 1;
			//aciUpdateCmdPacket(2);
			aciUpdateCmdPacket(0);
			//doBeep = 0;
		}
		else
			startStopMotor(motor_start);
	}

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

	unsigned char *tbyte = (unsigned char *) byte;
	for (int i = 0; i < cnt; i++) {
		write(fd, &tbyte[i], 1);
	}
}

void *aciThread(void) {
	int result = 0;
	int timecounter = 0;
	unsigned char data = 0;

	while (1) {
		result = read(fd, &data, 1);
		while (result > 0) {
			aciReceiveHandler(data);
			result = read(fd, &data, 1);
		}
		aciEngine();
		usleep(10000);
	}
	return NULL;
}

void varListUpdateFinished() {

}

void cmdListUpdateFinished() {
	printf("command list received!\n");

	aciAddContentToCmdPacket(0, 0x0500, &motor1);
	aciAddContentToCmdPacket(0, 0x0501, &motor2);
	aciAddContentToCmdPacket(0, 0x0502, &motor3);
	aciAddContentToCmdPacket(0, 0x0503, &motor4);
	aciAddContentToCmdPacket(1, 0x0600, &ctrl_mode);
	aciAddContentToCmdPacket(1, 0x0601, &ctrl_enabled);
	aciAddContentToCmdPacket(1, 0x0602, &disable_motor_onoff_by_stick);
	aciAddContentToCmdPacket(0, 0x1014, &doBeep);
	aciSendCommandPacketConfiguration(0, 0);
	aciSendCommandPacketConfiguration(1, 1);
	//aciSendCommandPacketConfiguration(2, 1);
	motor1 = 0;
	motor2 = 0;
	motor3 = 0;
	motor4 = 0;
	ctrl_mode=0;
	ctrl_enabled=1;
	disable_motor_onoff_by_stick=1;
	doBeep = 0;
	aciUpdateCmdPacket(0);
	aciUpdateCmdPacket(1);
	//aciUpdateCmdPacket(2);
	cmd_ready=1;
}

void paramListUpdateFinished() {

}

