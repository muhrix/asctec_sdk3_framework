#include "aci_remote_v100/asctecCommIntf.h"

#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h>  // File control definitions
#include <sys/stat.h> // File permissions
#include <termios.h> // POSIX terminal control definitionss
#include <pthread.h>
#include <time.h>

int fd;
int data_fd;

unsigned char ctrl_mode = 0;
unsigned char ctrl_enabled = 0;
unsigned char disable_motor_onoff_by_stick = 0;

unsigned char par_ready = 0;
unsigned char var_ready = 0;
unsigned char cmd_ready = 0;

unsigned short bat = 0;
unsigned short choose = 1;

void transmit(void* byte, unsigned short cnt);
void varListUpdateFinished();
void cmdListUpdateFinished();
void paramListUpdateFinished();
void *aciThread(void);

int writeOnDevice(void *data, int bytes);
int readFromDevice(void *data, int bytes);
void resetRW();
void openDevice();

int main(int argc, char *argv[]) {
	pthread_t p_acithread;

	fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	struct termios port_settings; // structure to store the port settings in

	cfsetispeed(&port_settings, B57600); // set baud rates
	port_settings.c_cflag = B57600 | CS8 | CREAD | CLOCAL;
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

    openDevice();
    aciSetWriteHDCallback(writeOnDevice);
    aciSetReadHDCallback(readFromDevice);
    aciSetResetHDCallback(resetRW);

	pthread_create(&p_acithread, NULL, aciThread, NULL);

	aciGetDeviceVariablesList();
	printf("Waiting for variable list...\n");
	while(!var_ready) usleep(1000);
	aciGetDeviceCommandsList();
	printf("Waiting for command list...\n");
	while(!cmd_ready) usleep(1000);
	aciGetDeviceParametersList();
	printf("Waiting for parameter list...\n");
	while(!bat) usleep(1000);
	while(choose) {
		printf("First battery warning level: %dmV\n",bat);
		printf("Type in new battery warning voltage in mV (0=quit, 1=store on Device): ");
		scanf("%d",&choose);
		if(choose==1) {
			aciSendParamStore();
			printf("Parameter stored\n");
		}
		else if(choose!=0) {
			bat = choose;
			aciUpdateParamPacket(0);
		}
	}
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
	printf("variable list getted!\n");
	var_ready=1;
}

void cmdListUpdateFinished() {
	printf("command list getted!\n");
	cmd_ready=1;
}

void paramListUpdateFinished() {
	printf("parameter list getted!\n");
	aciAddContentToParamPacket(0,0x0001,&bat);
	aciSendParameterPacketConfiguration(0);
}

void openDevice() {
	data_fd = open("data",O_RDWR | O_CREAT, S_IRUSR | __S_IWRITE | S_IRGRP | S_IROTH);
}

int writeOnDevice(void *data, int bytes) {
	return write(data_fd,data,bytes);
}

int readFromDevice(void *data, int bytes) {
	return read(data_fd,data,bytes);
}

void resetRW() {
	close(data_fd);
	data_fd = open("data",O_RDWR | O_CREAT, S_IRUSR | __S_IWRITE | S_IRGRP | S_IROTH);
}


