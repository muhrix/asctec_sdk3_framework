#include "aci_remote_v100/asctecCommIntf.h"

#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h>  // File control definitions
#include <termios.h> // POSIX terminal control definitionss
#include <pthread.h>

int fd;

void transmit(void* byte, unsigned short cnt);
void versions(struct ACI_INFO);
void *aciThread(void);

int main(int argc, char *argv[]) {

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
	aciInfoPacketReceivedCallback(&versions);
	aciSetEngineRate(100, 10);

	pthread_create( &p_acithread, NULL, aciThread, NULL);
	aciCheckVerConf();
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
	int result = 0;
	unsigned char data = 0;
	while(1)
	{
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

void versions(struct ACI_INFO aciInfo) {
	printf("******************** Versions *******************\n");
	printf("* Type\t\t\tDevice\t\tRemote\t*\n");
	printf("* Major version\t\t%d\t=\t\%d\t*\n",aciInfo.verMajor,ACI_VER_MAJOR);
	printf("* Minor version\t\t%d\t=\t\%d\t*\n",aciInfo.verMinor,ACI_VER_MINOR);
	printf("* MAX_DESC_LENGTH\t%d\t=\t\%d\t*\n",aciInfo.maxDescLength,MAX_DESC_LENGTH);
	printf("* MAX_NAME_LENGTH\t%d\t=\t\%d\t*\n",aciInfo.maxNameLength,MAX_NAME_LENGTH);
	printf("* MAX_UNIT_LENGTH\t%d\t=\t\%d\t*\n",aciInfo.maxUnitLength,MAX_UNIT_LENGTH);
	printf("* MAX_VAR_PACKETS\t%d\t=\t\%d\t*\n",aciInfo.maxVarPackets,MAX_VAR_PACKETS);
	printf("*************************************************\n");
}

