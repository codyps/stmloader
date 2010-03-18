#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <termios.h>
#include <unistd.h>

int debug = 1;

#define LOG(...) do {                               \
		if (debug) {                        \
			fflush(stdout);             \
			fprintf(stderr,__VA_ARGS__);\
			fflush(stderr);             \
		}                                   \
	} while(0);                                 \

#define WARN(_exitnum_,_errnum_,...) do {                          \
		fflush(stdout);                                    \
		fprintf(stderr,"%s:%d ",__func__,__LINE__);        \
		if (_errno_)                                       \
			fprintf(stderr,"[%s] : ",strerror(errnum));\
		else                                               \
			fprintf(stderr," : ");                     \
		fprintf(stderr,__VA_ARGS__);                       \
		fflush(stderr);                                    \
		if (_exitnum_)                                     \
			exit(_exitnum_);                           \
	} while(0);

void perror_at_line(int status, int errnum, const char *fname,
	unsigned int linenum, const char *format, ...) {
	va_list vl;
	va_start(vl,format);

	fflush(stdout);
	fprintf(stderr,"%s:%d ",__func__,__LINE__);
	if (errnum)
		fprintf(stderr,"[%s] : ",strerror(errnum));
	else
		fprintf(stderr," : ");
	vfprintf(stderr,format,vl);                       
	fflush(stderr);                                   
	if (status)                                     
		exit(status);
}

enum to_boot {
	c_get      = 0x00,
	c_getv     = 0x01,
	c_get_id   = 0x02,
	c_read     = 0x11,
	c_run      = 0x21,
	c_write    = 0x31,
	c_erase    = 0x43,
	c_w_prot   = 0x63,
	c_w_unprot = 0x73,
	c_r_prot   = 0x82,
	c_r_unprot = 0x92,

	i_start    = 0x7F
};

enum from_boot {
	b_ack   = 0x79,
	b_nack  = 0x1F
};

/*
 * usart even parity
 * 115200 baud
 */

/*
 * checksum: all received bytes are XORed. A byte containing
 * the computed XOR of all previous bytes is added to the
 * end of each communication (checksum byte). By XORing all
 * received bytes, data + checksum, the result at the end of
 * the packet must be 0x00.
 */

enum returns{
	kERR = -3,
	kUNEX = -2,
	kTIME = -1,
	kACK = 0,
	kNACK = 1
};

// -3 = error
int s_read(int fd, void *buf, size_t nbyte, long usec_tout) {
	size_t pos = 0;
	ssize_t ret;
	size_t sret;
	fd_set fds;
	struct timeval timeout = { .tv_sec=0, .tv_usec=0 };

	do {
		FD_ZERO(&fds);
		FD_SET(fd,&fds);
		timeout.tv_sec =0;
		timeout.tv_usec=usec_tout;

		sret = select(fd+1,&fds,NULL,NULL,&timeout);
		if (sret == 0) return kTIME;

		if (sret != 1) {
			perror("select");
			return kERR;
		}
	
		ret = read(fd, buf + pos, nbyte - pos);
		if (ret == -1) {
			perror("s_read");
			return kERR;
		}
		pos += ret;
	
	} while (pos < nbyte);
	return 0;
}


// 1 = nack, 0 = ack, -1 = timeout, -3 = error
int wait_ack(int fd, long usec_tout) {
	char tmp;
	int ret;
	fd_set fds;
	struct timeval timeout = { .tv_sec=0, .tv_usec=usec_tout };
	do {
		FD_ZERO(&fds);
		FD_SET(fd,&fds);

		ret = select(fd+1,&fds,NULL,NULL,&timeout);

		switch (ret) {
			case 0: LOG("   wait_ack timeout\n");
				return -1;
	
			case 1: 
				read(fd,&tmp,1);
				if (tmp == b_ack) {
					LOG("   got ack\n");
					return 0;
				}
				if (tmp == b_nack) {
					LOG("   got nack\n");
					return 1;
				}
				LOG("   wait_ack recieved junk byte %x\n",tmp);
				break;
			default:
			case -1:
				LOG("wait_ack: error");
				return kERR;
		}
	} while (1);
}

// 0 = success, 1 = nacked, -1 = timeout, -2 = unknowndata, -3 = error
int bootloader_init(int fd, long usec_tout) {
	char tmp = i_start;
	int ret;
	do {
		do {
			ret = write(fd,&tmp,1);
		} while (ret == 0);
		if (ret == -1) return kERR - 1;
		ret = wait_ack(fd,usec_tout);
	} while( ret < 0);
	return ret;
}

int send_command(int fd, enum to_boot com, long usec_tout) {
	char tmp[2];
	ssize_t ret;
	size_t pos = 0;
	tmp[0] = com;
	tmp[1] = ~com;
	do {
		ret = write( fd, tmp + pos, sizeof(tmp) - pos);
		if (ret == -1)
			return kERR - 1;
		pos += ret;
	} while(pos < sizeof(tmp));
	
	return wait_ack(fd, usec_tout);
}

void usage(char *name) {
	fprintf(stderr,"usage: %s <serial port>\n",name);
}

int serial_init(int fd) {
	struct termios tp_o;
	struct termios tp_n;
	int ret = tcgetattr(fd, &tp_o);
	if (ret == -1) {
		perror("tcgetattr");
		return 3;
	}

	tp_n = tp_o;

	tp_n.c_iflag = INPCK | IXON | IXOFF;
	tp_n.c_oflag = 0;
	tp_n.c_cflag = CS8 | CREAD | PARENB | CLOCAL;
	tp_n.c_lflag = 0;

	ret = cfsetispeed(&tp_n,B115200);
	if (ret == -1) {
		perror("cfsetispeed");
		return 4;
	}
	ret = cfsetospeed(&tp_n,B115200);
	if (ret == -1) {
		perror("cfsetospeed");
		return 5;
	}

	ret = tcsetattr(fd, TCSAFLUSH, &tp_n);
	if (ret == -1) {
		perror("tcsetattr");
		return 6;
	}

	return 0;
}
int get_id(int fd, long utimeout) {
	int ret;
	do {
		ret = send_command(fd, c_getv, utimeout);
	} while (ret == kTIME);
	LOG("send command c_geti (%d)\n",ret);
	if (ret <= kERR) {
		perror("send command");
		return ret -1;
	}

	char len;
	ret = s_read(fd, &len, 1, utimeout);

	char *data = malloc(len+1);
	ret = s_read(fd, data, len+1, utimeout);

	wait_ack(fd,utimeout);

	printf("GET_ID\n"
	       " PID: %02x " 
	      ,data[0]);
	int i;
	for (i = 0; i <	len; i++) 
		printf("%02x ",data[i+1]);
	putchar('\n');
	return 0;
}


int get_version(int fd, long utimeout) {
	int ret;
	do {
		ret = send_command(fd, c_getv, utimeout);
	} while (ret == kTIME);
	LOG("get_version: command c_getv (%d)\n",ret);
	if (ret <= kERR) {
		perror("send command");
		return ret -1;
	}

	char data[3];
	ret = s_read(fd, data, 3, utimeout);

	wait_ack(fd,utimeout);

	printf("GETV\n"
	       " bootloader version: %x\n"
	       " option byte 1 (0) : %x\n"
	       " option byte 2 (0) : %x\n"
	      ,data[0],data[1],data[2]);
	return 0;
}

int get_commands(int fd, long utimeout) {
	int ret;
	ret = send_command(fd, c_get_id, utimeout);
	LOG("sent command c_get (%d)\n",ret);
	if (ret < 0) {
		fprintf(stderr,"get_c: send_command : %d : ",ret);
		perror(0);
		return ret - 1;
	}

	uint8_t n;
	ret = s_read(fd, &n, 1, utimeout);
	LOG("s_read{c_get bytes}: %d [ numbytes: %u ]\n",ret,n);

	if (ret < 0)  {
		fprintf(stderr,"get_c: s_read : %d : ", ret);
		perror(0);
		return ret - 2;
	}

	uint8_t *get_d = malloc(n);
	if (!get_d) {
		perror("malloc");
		return -7;
	}
	
	ret = s_read(fd, get_d, n, utimeout);
	if (ret < 0) {
		fprintf(stderr,"get_c: s_read: %d : ",ret);
		perror(0);
		return ret -3;
	}
	LOG("s_read{c_get data}: %d\n",ret);

	ret = wait_ack(fd,utimeout);
	if (ret) {
		fprintf(stderr,"get_c: wait_ack: %d : ",ret);
		perror(0);
	}
	size_t i;
	printf("GET\n"
	       " bootloader version: %x\n"
	       " supported commands: ",get_d[0]);
	for (i = 1; i < n; i++) {
		printf("%x, ",get_d[i]);
	}
	putchar('\n');
	return 0;
}

int main(int argc, char **argv) {
	long utimeout = 200000;
	if (argc != 2) {
		usage(argv[0]);
		return 1;
	}
	
	int serial_fd = open(argv[1], O_RDWR);//fileno(serial_f);
	if (serial_fd == -1) {
		perror("Failed to open serial");
		return 2;
	}

	int ret = serial_init(serial_fd);
	if (ret) {
		fprintf(stderr,"could not initialize serial\n");
	}

	do {
		ret = bootloader_init(serial_fd, utimeout);
		//printf("bootloader_init: %d\n",ret);
		if (ret <= kERR) {
			fprintf(stderr,"bootloader_init (%d):",ret);
			perror(0);
			return ret;
		}
	} while (ret < 0);

	printf("connected to bootloader (%d)\n",ret);

	get_commands(serial_fd,utimeout);
	get_version(serial_fd,utimeout);
	get_id(serial_fd,utimeout);
	return 0;
}
