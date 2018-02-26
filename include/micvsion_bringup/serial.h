#ifndef _SERIAL_H
#define _SERIAL_H

#include <stdint.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <string.h>
#include <errno.h>

#define BUFFER_SIZE 256
#define TIME_OUT 20 // 20 ms

enum parity
{
	NO,
	EVEN,
	ODD
};

class myserial
{
	private:
		
		int	m_fd;
		struct termios m_oldtio;
		struct termios m_newtio;

	
	public:
	    bool stop_thread;
		char Open(char* port, int baud, char bits, parity parity, char stopbit);
		void Close(void);
		char Write(unsigned char* buffer, int length);
		int Read(unsigned char* buffer);
	    int Read(unsigned char* buffer, int len, int timeouts);
		int ReadByte(char byte);
};

#endif
