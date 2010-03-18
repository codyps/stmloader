TARGET = stmboot

SOURCE= $(wildcard *.c)
HEADER = $(wildcard *.h)

OBJECTS= $(SOURCE:=.o)
TRASH  = gmon.out

CFLAGS = -g
ALL_CFLAGS = -Wall $(CFLAGS)
ALL_LFLAGS = $(ALL_CFLAGS) $(LDFLAGS)
ALL_AFLAGS = $(ALL_CFLAGS)

CC = gcc
AS = gcc
LD = gcc
RM = rm -f

.SUFFIXES:
.SUFFIXES: .c .o .s

all: $(TARGET)


$(TARGET) : $(OBJECTS)
	$(LD) $(ALL_LFLAGS) -o $@ $^

clean:
	$(RM) $(TARGET) $(OBJECTS) $(TRASH)

%.c.o : %.c $(HEADER)
	$(CC) $(ALL_CFLAGS) -c -o $@ $<

%.s.o : %.s
	$(AS) $(ALL_AFLAGS) -c -o $@ $<

.PHONY: all clean

