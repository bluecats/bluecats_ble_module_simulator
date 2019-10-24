/*
 ============================================================================
 Name        : ble_module_simulator.c
 Author      : Bluecats
 Version     :
 Copyright   : Bluecats C
 Description : Hello World in C, Ansi-style
 ============================================================================
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <errno.h>
#include <time.h>

typedef struct serialApiPDUHdr_t
{
	unsigned char pduType;
	unsigned char clsId;
	unsigned char cmdOrEvt;
	unsigned char payLen;
	unsigned char payCRC8;
	unsigned char hdrCRC8;
} __attribute__ ((packed)) serialApiPDUHdr_t;

typedef struct serialApiPDU_t
{
	serialApiPDUHdr_t hdr;
	unsigned char payData[];
} __attribute__ ((packed)) serialApiPDU_t;

int util_packet_to_hex_string(void* packet, int len, char sep, char buf[])
{
	unsigned char* bytes = (unsigned char*)packet;
	int i = 0;
	int curLen = 0;
	while(i < len){
		if(sep == '\0')
			curLen += sprintf(buf + curLen, "%02X", bytes[i]);
		else
			curLen += sprintf(buf + curLen, "%02X%c", bytes[i], sep);
		i++;
	}
	return curLen;
}

void util_packet_reverse(unsigned char *packet, int len)
{
	for(int i =0; i < len/2; i++)
	{
		char left = packet[i];
		packet[i] = packet[len - i -1];
		packet[len - i - 1] = left;
	}
}

static int serial_init(FILE *serialPort) {

	struct termios opt;
	int fd = fileno(serialPort);

	if (tcgetattr(fd, &opt) != 0) {
		printf("Error from tcgetattr\n");
		return 0;
	}

	//speed_t speed = B9600;
	speed_t speed = B921600;
	if(cfsetispeed(&opt, speed) == -1){
		printf("Bad baud  %u\n",speed);
		return 0;
	}
	cfsetispeed(&opt, speed); //why twice?

	opt.c_cflag |= (CLOCAL | CREAD);    // ignore modem controls
	opt.c_cflag &= ~CSIZE;
	opt.c_cflag |= CS8;         // 8-bit characters
	opt.c_cflag &= ~PARENB;     // no parity bit
	opt.c_cflag &= ~CSTOPB;     // only need 1 stop bit
	opt.c_cflag &= ~CRTSCTS;    // no hardware flowcontrol

	// setup for non-canonical mode
	opt.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	opt.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	opt.c_oflag &= ~OPOST;

	opt.c_cc[VMIN] = 0;
	opt.c_cc[VTIME] = 0;

	if (tcsetattr(fd, TCSANOW, &opt) != 0) {
		printf("Error from tcsetattr\n");
		return 0;
	}
	return 1;
}

static void serial_write(FILE *sp, unsigned char *bytes, int len) {

	int numW = 0;
	while(numW < len) {

		int n = write(fileno(sp), bytes+numW, len-numW);
		if(n < 0) {
			printf("Error writing data %s\n", strerror(errno));
			exit(0);
		} else {
			//printf("Wrote %i bytes\n", n);
		}
		numW+=n;
	}
}

static unsigned char serial_api_crc8(const void* vptr, int len)
{
	const unsigned char *data = vptr;
	unsigned crc = 0;
	int i, j;
	for (j = len; j; j--, data++) {
		crc ^= (*data << 8);
		for(i = 8; i; i--) {
			if (crc & 0x8000)
				crc ^= (0x1070 << 3);
			crc <<= 1;
		}
	}
	return (unsigned char)(crc >> 8);
}

int main(int argc, char **argv) {

	srand(time(NULL));

	if(argc < 3) {
		printf("Requires serial port, total ads, num devs, ad rate millis as arguments\n");
		return 0;
	}

	char *serialPortPath = argv[1];

	printf("Serial Port is %s \n", serialPortPath);

	FILE *sp = fopen(serialPortPath, "r+");

	if (!sp) {
		printf("Could not open serial port %s\n", serialPortPath);
		return 0;
	}

	if(!serial_init(sp)){
		printf("Failed to initialize serial\n");
		return 0;
	}

	int NUM_ADS_TOTAL = strtol(argv[2], NULL, 10);
	int NUM_DEVICES = strtol(argv[3], NULL, 10);
	int SLEEP_TIME_MILLIS = strtol(argv[4], NULL, 10);

	//unsigned char testAd[] = {0x80, 0xBC, 0x10, 0x27, 0xE7, 0x3B, 0xB2, 0xF9,
	//			0x05, 0x2D, 0x07, 0x98, 0xB8, 0x25, 0x02, 0x01, 0x06, 0x03, 0x03,
	//			0xAA, 0xFE, 0x17, 0x16, 0xAA, 0xFE, 0x00, 0xF6, 0xAA, 0xFF, 0xAA,
	//			0xFF, 0xAA, 0xFF, 0xAA, 0xFF, 0xAA, 0xFF, 0x00, 0x00, 0x00, 0x01,
	//			0xFB, 0x6A, 0x00, 0x00};

	unsigned char exEddy[] = {0x02, 0x01, 0x06, 0x03, 0x03,
			0xAA, 0xFE, 0x17, 0x16, 0xAA, 0xFE, 0x00, 0xF6, 0xAA, 0xFF, 0xAA,
			0xFF, 0xAA, 0xFF, 0xAA, 0xFF, 0xAA, 0xFF, 0x00, 0x00, 0x00, 0x01,
			0xFB, 0x6A, 0x00, 0x00};

	int maxPDUSizeBytes = 50;

	serialApiPDUHdr_t hdr;
	hdr.pduType = 0x80;
	hdr.clsId = 0xbc;
	hdr.cmdOrEvt = 0x10;

	serialApiPDU_t *pdu = calloc(1, maxPDUSizeBytes);
	pdu->hdr = hdr;
	char channel = 37;
	signed char rssi = -70;

	const int payHdrLen = 8;

	//just do all eddystones for now
	memcpy(pdu->payData+payHdrLen, exEddy, sizeof(exEddy));

	char hexStr[512]="";
	//char mac[] = {0xB2, 0xF9, 0x05, 0x2D, 0x07, 0x98};


	long long i = 0;
	int bufLen = 1000;
	unsigned char *cBuf = calloc(1,bufLen);
	int bufIdx = 0;
	int forever = 0 || (NUM_ADS_TOTAL == 0);
	while(forever || (i < NUM_ADS_TOTAL)) {

		long mac = rand() % NUM_DEVICES;
		rssi = -1 * (rand() % 127);
		//rssi = rand() % 2 == 0 ? 0xb1 : 0xb5;
		//mac[5] = rand() % 2 == 0 ? 0x11 : 0x10;

		memcpy(pdu->payData,&mac,6);
		pdu->payData[6] = rssi;
		pdu->payData[7] = channel;

		//copy mac to end of eddy
		memcpy(pdu->payData+payHdrLen + 23, &mac, 6);

		//pay length
		int adLen = sizeof(exEddy);
		pdu->hdr.payLen = payHdrLen + adLen;

		//calc crc
		pdu->hdr.payCRC8 = serial_api_crc8(pdu->payData, pdu->hdr.payLen);
		pdu->hdr.hdrCRC8 = serial_api_crc8(pdu, sizeof(serialApiPDU_t)-1);

		//lets write some data
		int pduLen = sizeof(serialApiPDUHdr_t) + pdu->hdr.payLen;

		if(bufIdx+pduLen>bufLen){

			serial_write(sp, cBuf, bufIdx);
			printf("\n\nJust Wrote %i bytes: i=%lli\n\n", bufIdx,i);
			bufIdx = 0;
			usleep(1000 * SLEEP_TIME_MILLIS);
		}

		memcpy(cBuf+bufIdx, pdu, pduLen);
		bufIdx+=pduLen;

		util_packet_reverse((unsigned char *)&mac, 6);
		util_packet_to_hex_string(&mac, 6,' ',hexStr);
		printf("%s\n", hexStr);

		//util_packet_to_hex_string(pdu, totalLen,' ',hexStr);
		//printf("%s\n", hexStr);

		//sleep(1);
		i++;
	}
	usleep(100000);

	//write remaining bytes in buf
	printf("Writing remaining buffer\n");
	serial_write(sp, cBuf, bufIdx);

}
