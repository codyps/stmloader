#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <getopt.h>

int debug = 0;
#define unlikely(x)     __builtin_expect((x),0)

#define INFO(...) do {                                                     \
		if (unlikely(debug)) {                                     \
			fprintf(stderr,"INFO: ");                          \
			perror_at_line(0,0,__func__,__LINE__,__VA_ARGS__); \
		}                                                          \
	} while(0)

#define WARN(_exitnum_,_errnum_,...) do {               \
		fprintf(stderr,"WARN: ");               \
		perror_at_line(_exitnum_,_errnum_,      \
			__func__,__LINE__,__VA_ARGS__); \
	} while(0)

void __attribute__((format(printf,5,6)))
perror_at_line(int status, int errnum, const char *fname,
	unsigned int linenum, const char *format, ...)
{
	va_list vl;
	va_start(vl,format);

	fflush(stdout);
	fprintf(stderr,"%s:%d ",fname,linenum);
	if (errnum)
		fprintf(stderr,"[%s] : ",strerror(errnum));
	else
		fprintf(stderr," : ");
	vfprintf(stderr,format,vl);
	fputc('\n',stderr);
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

enum returns {
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
			WARN(0,errno,"select");
			return kERR;
		}
	
		ret = read(fd, buf + pos, nbyte - pos);
		if (ret == -1) {
			WARN(0,errno,"read failed");
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
	ssize_t rret;
	fd_set fds;
	do {
		struct timeval timeout = { .tv_sec=0, .tv_usec=usec_tout };
		FD_ZERO(&fds);
		FD_SET(fd,&fds);

		ret = select(fd+1,&fds,NULL,NULL,&timeout);

		switch (ret) {
			case  0: 
				INFO("timeout");
				return -1;
	
			case  1: 
				rret = read(fd,&tmp,1);
				if (rret == 1) {
					if (tmp == b_ack) {
						INFO("got ack");
						return 0;
					}
					if (tmp == b_nack) {
						INFO("got nack");
						return 1;
					}
					WARN(0,0,"recieved junk byte %x",tmp);
				} else {
					WARN(0,0,"read %d\n",rret);
				}
				break;
			default:
			case -1:
				WARN(0,errno,"select error");
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
	} while( ret < 0 );
	return ret;
}

int send_command(int fd, enum to_boot com, long usec_tout) {
	char tmp[2];
	ssize_t ret;
	size_t pos = 0;
	INFO("sending 0x%02X",com);
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
int serial_init(int fd) {
	struct termios tp_o;
	struct termios tp_n;
	int ret = tcgetattr(fd, &tp_o);
	if (ret == -1) {
		WARN(0,errno,"tcgetattr");
		return 3;
	}

	tp_n = tp_o;

	tp_n.c_iflag = INPCK | IXON | IXOFF;
	tp_n.c_oflag = 0;
	tp_n.c_cflag = CS8 | CREAD | PARENB | CLOCAL;
	tp_n.c_lflag = 0;

	ret = cfsetispeed(&tp_n,B115200);
	if (ret == -1) {
		WARN(0,errno,"cfsetispeed");
		return 4;
	}
	ret = cfsetospeed(&tp_n,B115200);
	if (ret == -1) {
		WARN(0,errno,"cfsetospeed");
		return 5;
	}

	ret = tcsetattr(fd, TCSAFLUSH, &tp_n);
	if (ret == -1) {
		WARN(0,errno,"tcsetattr");
		return 6;
	}

	return 0;
}
int get_id(int fd, long utimeout) {
	int ret;
	do {
		ret = send_command(fd, c_get_id, utimeout);
	} while (ret == kTIME);
	INFO("send command c_geti (%d)",ret);
	if (ret) {
		WARN(0,errno,"send command");
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
	INFO("getting version\n");
	int ret;
	ret = send_command(fd, c_getv, utimeout);
	INFO("send command c_getv (%d)",ret);
	if (ret) {
		WARN(0,errno,"send command");
		return ret -1;
	}

	char data[3];
	ret = s_read(fd, data, 3, utimeout);
	if (ret) {
		WARN(0,errno,"s_read of 3 bytes returned %d",ret);
		return -4;
	}
		
	ret = wait_ack(fd,utimeout);
	if (ret) {
		WARN(0,errno,"wait_ack returned %d",ret);
	}

	printf("GETV\n"
	       " bootloader version: %x\n"
	       " option byte 1 (0) : %x\n"
	       " option byte 2 (0) : %x\n"
	      ,data[0],data[1],data[2]);
	return 0;
}

int get_commands(int fd, long utimeout) {
	int ret;
	do {
		ret = send_command(fd, c_get, utimeout);
	} while (ret == kTIME);
	INFO("sent command c_get (%d)",ret);
	if (ret) {
		WARN(0,errno,"send_command returned %d",ret);
		return ret - 1;
	}

	uint8_t n;
	ret = s_read(fd, &n, 1, utimeout);
	INFO("s_read of numbytes returned %d and got %u",ret,n);

	if (ret)  {
		WARN(0,errno,
			"s_read of numbytes returned %d and got %u",ret,n);
		return ret - 2;
	}

	uint8_t *get_d = malloc(n);
	if (!get_d) {
		perror("malloc");
		return -7;
	}
	
	ret = s_read(fd, get_d, n, utimeout);
	if (ret) {
		WARN(0,errno,
			"s_read of commands returned %d",ret);
		return ret -3;
	}
	INFO("s_read{c_get data}: %d",ret);

	ret = wait_ack(fd,utimeout);
	if (ret) {
		WARN(0,errno,"wait_ack: %d",ret);
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

#define MSK(_msk_,_x_) !!((_msk_)&(_x_))

int tty_ctrl(int fd, int pin_msk, bool high) {
	int s;
	int r = ioctl(fd, TIOCMGET, &s);
	if (r)
		WARN(0,errno,"ioctl_TIOCMGET returned %d : \"%s\"",r,strerror(r));
	
	if (pin_msk) {
		int sb = s;
		if (high) {
			sb |= pin_msk;
		} else {
			sb &= pin_msk;
		}
		
		r = ioctl(fd, TIOCMSET, &sb);
		if (r)
			WARN(0,errno,"ioctl_TIOCMSET returned %d : \"%s\"",r,strerror(r));
	}
	return s;
}

void tty_printctrl(int fd) {
	int status = tty_ctrl(fd,0,0);
	
	printf("0x%02x :: ",status);
	printf("CAR:%d ",MSK(TIOCM_CAR,status));
	printf("RNG:%d ",MSK(TIOCM_RNG,status));
	printf("DSR:%d ",MSK(TIOCM_DSR,status));
	printf("DTR:%d ",MSK(TIOCM_DTR,status));
	printf("RTS:%d ",MSK(TIOCM_RTS,status));
	printf("CTS:%d ",MSK(TIOCM_CTS,status));
	#if defined(TIOCM_ST)
	printf("ST:%d " ,MSK(TIOCM_ST,status));
	#endif
	#if defined(TIOCM_SR)
	printf("SR:%d ",MSK(TIOCM_SR,status));
	#endif
	putchar('\n');
}

const char optstr[] = "hDt:icvprgweXxZzT";

void usage(char *name) {
	fprintf(stderr,
		"usage: %s [options] [actions] <serial port>\n"
		"options: -h            help (show this)\n"
		"         -D            debugging output\n"
		"         -t useconds   change serial timeout\n"
		"actions: -i            initialize bootloader\n"
		"         -c            get boot supported commands\n"
		"         -v            get boot version\n"
		"         -p            get pid\n"
		"         -r            read memory\n"
		"         -g            \"go\", execute\n"
		"         -w            write memory\n"
		"         -e            erase memory\n"
		"         -X sector:ct  write protect\n"
		"         -x            write unprotect\n"
		"         -Z            readout protect\n"
		"         -z            readout unprotect\n"
		"         -T            just read ctrl lines\n"
		       ,name);
}

int main(int argc, char **argv) {
	long utimeout = 200000;

	if (argc < 2) {
		WARN(0,0,"unspecified serial port\n");
		usage(argv[0]);
		return 1;
	}

	char *serial_s = argv[argc-1];
	int serial_fd = open(serial_s, O_RDWR);//fileno(serial_f);
	if (serial_fd == -1) {
		WARN(-2,errno,"failed to open serial port \"%s\"",serial_s);
		return 2;
	}
	int ret = serial_init(serial_fd);
	if (ret) {
		WARN(-1,errno,"could not initialize serial \"%s\", %x",serial_s,ret);
	}

	int opt;
	while ( (opt = getopt(argc,argv,optstr)) != -1 ) {
		switch(opt) {
		case '?':
			WARN(-1,0,"bad option %c",optopt);
		case 'h':
			usage(argv[0]);
			return 1;
		default:
			WARN(-1,0,"unknown option %c",optopt);

		case 'T':
			do {
				tty_printctrl(serial_fd);
				usleep(utimeout);
			} while(1);
			return 0;

		case 'D':
		case 'd':
			debug = 1;
			INFO("debuging enabled");
			break;
		case 't': { 
				long tmp;
				int ret = sscanf(optarg,"%li",&tmp);
				if (ret != 1) {
					WARN(-2,0,"specified timeout (\"%s\") invalid",optarg); 
				}
				utimeout = tmp;
				INFO("timeout changed to %li usecs",utimeout);
			}
			break;
		case 'i':
			INFO("connecting to bootloader....");
			do {
				ret = bootloader_init(serial_fd, utimeout);
				//printf("bootloader_init: %d\n",ret);
				if (ret <= kERR) {
					WARN(ret,errno,"bootloader_init: %d",ret);
				}
			} while (ret < 0);
			INFO("connected to bootloader : %d",ret);
			break;	
		case 'c':
			get_commands(serial_fd,utimeout);
			break;
		case 'v':
			get_version(serial_fd,utimeout);
			break;
		case 'p':
			get_id(serial_fd,utimeout);
			break;
		}
	}

	if ( (argc - optind) != 1) {
		WARN(0,0,"Serial port is unspecified");
		usage(argv[0]);
		return 1;
	}


	return 0;
}
