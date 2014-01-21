#include "aci_remote_v100/asctecCommIntf.h"

#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h>  // File control definitions
#include <termios.h> // POSIX terminal control definitionss
#include <pthread.h>

int fd;

int32_t angle_pitch;
int32_t angle_roll;
int32_t angle_yaw;

int16_t motor_rpm_0;
int16_t motor_rpm_1;
int16_t motor_rpm_2;
int16_t motor_rpm_3;

unsigned char var_getted;

void transmit(void* byte, unsigned short cnt);
void varListUpdateFinished(void);
void *aciThread(void);

int main(int argc, char *argv[]) {

	var_getted=0;
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

	cfsetispeed(&port_settings, B57600);    // set baud rates
	port_settings.c_cflag = B57600 | CS8 | CREAD | CLOCAL;
	port_settings.c_iflag = IGNPAR;
	port_settings.c_oflag = 0;
	port_settings.c_lflag = 0;
	tcsetattr(fd, TCSANOW, &port_settings); // apply the settings to the port


	aciInit();
	aciSetSendDataCallback(&transmit);
	aciSetVarListUpdateFinishedCallback(&varListUpdateFinished);
	aciSetEngineRate(100, 10);

	pthread_create( &p_acithread, NULL, aciThread, NULL);
	aciGetDeviceVariablesList();
	pthread_join( p_acithread, NULL);
}

void transmit(void* byte, unsigned short cnt) {

	unsigned char *tbyte = (unsigned char *)byte;
	for (int i = 0; i < cnt; i++) {
		write(fd, &tbyte[i], 1);
	}
}

void *aciThread(void)
{
	int timecounter=0;
	int result = 0;
	unsigned char data = 0;
	while(1)
	{
		timecounter++;
		result = read(fd, &data, 1);
		while (result > 0) {
			aciReceiveHandler(data);
			result = read(fd, &data, 1);
		}
		if(timecounter>100) {
			if(var_getted) {
				printf("Angles before Synchronizing: %f\t%f\t%f\n", (double)(angle_pitch)/1000, (double)(angle_roll)/1000 , (double)(angle_yaw)/1000);
				aciSynchronizeVars();
				printf("Angles after Synchronizing: %f\t%f\t%f\n---------\n", (double)(angle_pitch)/1000, (double)(angle_roll)/1000 , (double)(angle_yaw)/1000);
				printf("Motor RPM: %d\t%d\t%d\t%d\n", motor_rpm_0, motor_rpm_1, motor_rpm_2, motor_rpm_3);
			}
			timecounter=0;
		}
		aciEngine();
		usleep(10000);
	}
	return NULL;
}

void varListUpdateFinished(void) {
	printf("variables updated\n");
	aciAddContentToVarPacket(0,0x0300,&angle_pitch);
	aciAddContentToVarPacket(0,0x0301,&angle_roll);
	aciAddContentToVarPacket(0,0x0302,&angle_yaw);
	aciAddContentToVarPacket(0,0x0100,&motor_rpm_0);
	aciAddContentToVarPacket(0,0x0101,&motor_rpm_1);
	aciAddContentToVarPacket(0,0x0102,&motor_rpm_2);
	aciAddContentToVarPacket(0,0x0103,&motor_rpm_3);
	aciSetVarPacketTransmissionRate(0,10);
	aciVarPacketUpdateTransmissionRates();
	aciSendVariablePacketConfiguration(0);
	var_getted=1;
}
