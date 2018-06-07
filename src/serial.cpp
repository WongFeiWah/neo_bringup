#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <neo_bringup/serial.h>
#include <neo_bringup/timeout.h>

char myserial::Open(char* port, int baud, char bits, parity parity, char stopbit) 
{
	char tmp[BUFFER_SIZE];
	
	// Convert to string
	sprintf(tmp, "%s", port);
	
	m_fd = open(tmp, O_RDWR | O_NOCTTY | O_NONBLOCK);
	
	// Check opening status 
	if (m_fd < 0)
	{
        printf("Error Opening %s Port\n", tmp);
		
        return 0;
	}
	//printf("sdkgflasdjf\n");can not jump into this 
	// Get terminal parameters
	tcgetattr(m_fd, &m_newtio);
	tcgetattr(m_fd, &m_oldtio);

	// Flushes data received but not read		
	ioctl(m_fd, TCIFLUSH);

	// Set baud rate (in and out)
	switch(baud)
	{
		case 9600	: cfsetspeed(&m_newtio, B9600)	; break;
		case 19200	: cfsetspeed(&m_newtio, B19200)	; break;
		case 38400	: cfsetspeed(&m_newtio, B38400)	; break;
		case 57600	: cfsetspeed(&m_newtio, B57600)	; break;
		case 115200	: cfsetspeed(&m_newtio, B115200); break;
		default 	: cfsetspeed(&m_newtio, B9600)	; break;
	}

	// Set byte size
	m_newtio.c_cflag &= ~CSIZE;	

	switch(bits)
	{
		case 5	: m_newtio.c_cflag |= CS5; break;
		case 6	: m_newtio.c_cflag |= CS6; break;
		case 7	: m_newtio.c_cflag |= CS7; break;
		case 8	: m_newtio.c_cflag |= CS8; break;
		default : m_newtio.c_cflag |= CS8; break;
	}

	// Set parity
	switch(parity)
	{
		case NO	  :
				m_newtio.c_cflag &=~ PARENB;	// Disable parity
			break;
			
		case EVEN :
				m_newtio.c_cflag |= PARENB;		// Enable parity
				m_newtio.c_cflag &= ~PARODD;	// Disable odd parity
			break;
			
		case ODD  :
				m_newtio.c_cflag |= PARENB;		// Enable parity
				m_newtio.c_cflag |= PARODD;		// Enable odd parity
			break;
			
		default	  :
				m_newtio.c_cflag &=~ PARENB;	// Disable parity
	}
	
	// Set stop bit
	switch(stopbit)
	{
		case 1: m_newtio.c_cflag &=~ CSTOPB	; break;	// Disable 2 stop bits
		case 2: m_newtio.c_cflag |= CSTOPB	; break;	// Enable 2 stop bits
		default: m_newtio.c_cflag &=~ CSTOPB;
	}

	// Enable receiver (CREAD) and ignore modem control lines (CLOCAL)
	m_newtio.c_cflag |= (CREAD | CLOCAL); 
	
	// Disable, canonical mode (ICANON = 0), echo input character (ECHO) and signal generation (ISIG)
	m_newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
	
	// Disable input parity checking (INPCK)
	m_newtio.c_iflag &= ~INPCK; 		  

	// Disable XON/XOFF flow control on output and input (IXON/IXOFF), framing and parity errors (IGNPAR), and disable CR to NL translation
	m_newtio.c_iflag &= ~(IXON | IXOFF | IXANY | IGNPAR | ICRNL);

	// Disable implementation-defined output processing (OPOST)
	m_newtio.c_oflag &=~ OPOST;	

	// Set terminal parameters
	tcsetattr(m_fd, TCSAFLUSH, &m_newtio);
	
	// Display settings
	printf("%s | BaudRate: %d | Bits: %d | Parity: %d | StopBits: %d\n", tmp, baud, bits, parity, stopbit);
    this->stop_thread = false;
	return 1;
}

void myserial::Close(void) 
{
	// Set old parameters
	tcsetattr(m_fd, TCSANOW, &m_oldtio);
    this->stop_thread = true;
	// Close serial port
	if(m_fd > 0)
	{
		close(m_fd);
	}
}


char myserial::Write(unsigned char* buffer, int length) 
{
	ssize_t r;
	const uint8_t *pos = (const uint8_t*)buffer;

	// Send data
	while(length > 0)
	{
		r = write(m_fd, pos, length);
		
		if(r < 1)
		{
			return 0;
		}

		length -= r;
		pos += r;
	}

	return 1;
}

int myserial::Read(unsigned char* buffer)
{
	ssize_t r = 1;
	int length = 0;
	char ch;

	// Set timeout between 2 bytes (20ms)
	timeout timeout(TIME_OUT);
	
	//memset(buffer, 0x00, BUFFER_SIZE);
	
	while((r > 0) || timeout.end())
	{
		// Protect buffer
		if(length >= (BUFFER_SIZE - 1)) return length;
		
		r = read(m_fd, &ch, 1);
		*(buffer + length) = (unsigned char)ch;			
		if(r > 0)
		{
			length++;

			// Start timer
			timeout.start();
		}
	}
	
	return length;
}

int myserial::Read(unsigned char* buffer, int len, int timeouts)
{
	ssize_t r = 1;
	int length = 0;
	char ch;

	// Set timeout between 2 bytes (20ms)
	timeout timeouter(timeouts);

	//memset(buffer, 0x00, BUFFER_SIZE);

	while((r > 0) || timeouter.end())
	{
		// Protect buffer
		if(length >= len) return length;

		r = read(m_fd, &ch, 1);
		*(buffer + length) = (unsigned char)ch;
		if(r > 0)
		{
			length++;

			// Start timer
			timeouter.start();
		}
	}

	return length;
}

int myserial::ReadByte(char byte)
{
	int r = 0;
	char ch[2] = {0};
	while(r == 0)
	{
		r = read(m_fd, ch , 1);
	}
	
	byte = ch[0];
	return r;
}


