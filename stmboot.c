#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <termios.h>
#include <unistd.h>

enum to_boot {
	c_get      = 0x00,
	c_get_prot = 0x01,
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
	b_nack  = 0x1F,
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
int s_read(int fd, void *buf, size_t nbyte) {
	size_t pos = 0;
	ssize_t ret;
	do {
		ret = read(fd, buf + pos, nbyte - pos);
		if (ret == -1) {
			perror("s_read");
			return -1;
		}
		pos += ret;
	
	} while (pos < nbyte);
	return 0;
}

int wait_ack(FILE *f, size_t timeout) {
	char tmp;
	size_t i;
	int ret;
	for(i = 0; i < timeout; i++) {
		ret = fgetc(f);	
		if (ret == -1) 
			return -2;
		if (ret == b_ack) 
			return 0;
		if (ret == b_nack)
			return 1;

	}
	return -3;
}

int bootloader_init(FILE *f) {
	char tmp = i_start;
	int ret;
	do {	
		ret = fputc(i_start,f);
		if (ret == -1)
				return -4;
		ret = wait_ack(fd,2000);
	} while( ret < 0);
	return ret;
}

int send_command(FILE *f, enum to_boot com) {
	char tmp[2];
	ssize_t ret;
	size_t pos = 0;
	tmp[0] = com;
	tmp[1] = ~com;
	do {
		ret = write( fd, tmp + pos, sizeof(tmp) - pos);
		if (ret == -1)
			return -1;
		pos += ret;
	} while(pos < sizeof(tmp));
	
	return wait_ack(fd, 20000);
}

void usage(char *name) {
	fprintf(stderr,"usage: %s <serial port>\n",name);
}

int serial_init(FILE *f) {
	int fd = fileno(f);
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

int main(int argc, char **argv) {
	if (argc != 2) {
		usage(argv[0]);
		return 1;
	}
	
	FILE *serial_f = fopen(argv[1], "rw");
	if (serial_fd == -1) {
		perror("Failed to open serial");
		return 2;
	}

	int ret = serial_init(serial_f);
	if (ret) {
		fprintf(stderr,"could not initialize serial\n");
	}

	do {
		ret = bootloader_init(serial_f);
		//printf("bootloader_init: %d\n",ret);
	} while (ret == -3);
	printf("connected to bootloader (%d)\n",ret);

	do {
		ret = send_command(serial_f, c_get);
		//printf("sendcommand(c_get): %d\n",ret);
	} while (ret == -3);
	printf("sent command c_get (%d)\n",ret);

	uint8_t n;
	ret = s_read(serial_fd, &n, 1);
	printf("s_read{c_get bytes}: %d [ numbytes: %u ]\n",ret,n);
	if (ret < 0) 
		return ret;

	uint8_t *get_d = malloc(n);
	if (!get_d) {
		perror("malloc");
		return -7;
	}
	
	ret = s_read(serial_fd, get_d, n);
	printf("s_read{c_get data}: %d\n",ret);
	size_t i;
	printf("  version: %x\n"
	       "  supported commands: ",get_d[0]);
	for (i = 1; i < n; i++) {
		printf("%x, ",get_d[i]);
	}
	putchar('\n');
	
	return 0;
}
