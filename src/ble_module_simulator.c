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
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <pthread.h>
#include <error.h>
#include <sys/time.h>

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

void * upd_listen(void *args);
static long long g_recvd_total_bytes = 0;
static long long g_recvd_total_ads = 0;
static struct timeval g_recv_last;

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
		printf("Requires serial port, total ads, num devs, ad rate as arguments and listen port (optional)\n");
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
	int listen_port = 0;

	pthread_t threadId=0;
	if(argv[5]) {
		listen_port = strtol(argv[5], NULL, 10);
		int *i = malloc(sizeof(int));
		*i = listen_port;

		//start the listening thread
		pthread_create(&threadId, NULL, upd_listen, i);
		pthread_detach(threadId);
	}

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


	long long bytes_sent_total = 0;
	long long i = 0;

	//can make this longer to simulate writing multiple ads per send
	//hardcode to 64 to cli arg matches ad rate
	int bufLen = 64;

	unsigned char *cBuf = calloc(1,bufLen);
	int bufIdx = 0;
	int forever = 0 || (NUM_ADS_TOTAL == 0);

	struct timeval stop, start;
	gettimeofday(&start, NULL);

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

		//lets write some ads
		int pduLen = sizeof(serialApiPDUHdr_t) + pdu->hdr.payLen;

		if(bufIdx+pduLen>bufLen){

			serial_write(sp, cBuf, bufIdx);
			bytes_sent_total+=bufIdx;

			if(i%100 == 0) {
				printf("Sent %lli bytes: i=%lli, %.02f%%\r", bytes_sent_total,i, 100*(float)i/NUM_ADS_TOTAL);
				fflush(stdout);
			}
			bufIdx = 0;

			//rand sleep to simulate 'bursty' ads
			//should average to SLEEP_TIME_MILLIS
			long stime = rand() % (2*SLEEP_TIME_MILLIS);
			usleep(1000 * stime);
		}

		memcpy(cBuf+bufIdx, pdu, pduLen);
		bufIdx+=pduLen;

		util_packet_reverse((unsigned char *)&mac, 6);
		util_packet_to_hex_string(&mac, 6,' ',hexStr);

		i++;
	}

	//write remaining buffer...
	serial_write(sp, cBuf, bufIdx);
	bytes_sent_total+=bufIdx;
	gettimeofday(&stop, NULL);

	printf("Sent %lli bytes: i=%lli, %.02f%%\n", bytes_sent_total,i, 100*(float)i/NUM_ADS_TOTAL);

	if(threadId) {
		//sleep for recv thread to catch up (if it can)
		//hardcode 1 second for network latency (rest is 'dropped')
		printf("Done! Give recv thread 2 seconds to catchup...Rest is considered 'dropped'\n");
		sleep(2);
		pthread_cancel(threadId);
		printf("\n");
	}

	double time_spent_millis = (double) (stop.tv_sec - start.tv_sec) * 1000 + (double) (stop.tv_usec - start.tv_usec) / 1000;
	double time_recvd_millis = (double) (g_recv_last.tv_sec - start.tv_sec) * 1000 + (double) (g_recv_last.tv_usec - start.tv_usec) / 1000;

	printf("----------------------------------\n");
	printf(
			"Test time:          %f\n"
			"Sent/Recvd ads      %lli/%lli\n"
			"Sent Ads/Sec        %f\n"
			"Recvd Ads/Sec       %f\n"
			"Sent/Recvd bytes    %lli/%lli\n"
			"Sent bytes/sec      %f\n"
			"Recvd bytes/sec     %f\n",
			time_spent_millis/1000,
			i,g_recvd_total_ads,
			(float)i/(time_spent_millis/1000),
			(float)g_recvd_total_ads/(time_recvd_millis/1000),
			bytes_sent_total, g_recvd_total_bytes,
			(float)bytes_sent_total/(time_spent_millis/1000),
			(float)g_recvd_total_bytes/(time_recvd_millis/1000)),
	printf("----------------------------------\n");
}

#define BUFSIZE 1024
void * upd_listen(void *args) {


	int sock, length, n;
	struct sockaddr_in server;
	struct sockaddr_in clientaddr;
	socklen_t clientlen;
	unsigned char buf[BUFSIZE];

	int port = *(int *)args;
	free(args);

	sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0) {
		printf("Error Opening socket\n");
		return 0;
	}

	length = sizeof(server);
	bzero(&server,length);
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port=htons(port);

	/* bind port to the IP */
	if (bind(sock, (struct sockaddr *) &server, length) < 0) {
		printf("Error binding\n");
		return 0;
	}

	printf("UDP listen port %i\n", port);
	while(1) {
		bzero(buf, BUFSIZE);
		n = recvfrom(sock, buf, BUFSIZE, 0, (struct sockaddr *) &clientaddr, &clientlen);
		if (n < 0)
			printf("ERROR in recvfrom\n");
		g_recvd_total_bytes+=n;
		g_recvd_total_ads++;
		gettimeofday(&g_recv_last, NULL);
	}

	return 0;

}
