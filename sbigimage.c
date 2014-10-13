/* -------------------------------------------------------------------
 * sbigimage.c  Open connection to /dev/ttyS0 (or whatever), send
 *              whatever commands are necessary to initialize the
 *              camera, up the Baud rate to the highest possible, then
 *              sit there and wait for a keystroke (i) to take an
 *              image.  Exit on (x).
 *
 * A. Hunter, 2013.3.29
 *
 *              I need to make this more flexible ...  considering
 *              convert the user interface to ncurses, so it's a
 *              little easier to use ... but that seems like kind of a
 *              project.
 *
 *              Now uses popen() to pipe some commands to gnuplot to
 *              plot the image immediately after I take it.  Ugly, but
 *              it's better than not seeing the image at all.
 *
 * A. Hunter, 2013.4.2
 * ------------------------------------------------------------------- */


#include <sys/select.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

/* #define MODEMDEVICE "/dev/cu.usbserial"  /\* Mac *\/ */
#define MODEMDEVICE "/dev/ttyS0"  /* Linux */
#define INITBAUD 9600  /* Baud rate on startup. */
#define INITBN B9600
#define HIGHBAUD 115200  /* Baud rate to try after startup. */
#define HIGHBN B115200
#define BUFLEN 1024  /* Buffer length in bytes. */
#define HEADOFFSET 163
#define TRUE 1
#define FALSE 0
#define OPEN 1
#define CLOSED 0
#define IMGFILE "image.dat"  /* Filename for image output.  This is
				a text file containing a space-delimited
				bitmap array. */


/* -------------------------------------------------------------------
   Opening connections to cpu and stdin:
   ------------------------------------------------------------------- */


/* -------------------------------------------------------------------
   Open /dev/ttyS0 for read/write.
   From the serial programming HOWTO:  "Open modem device for reading 
   and writing and NOT as controlling tty because we don't want to get
   killed if linenoise sends CTRL-C." */
void opencpu(int *fd, struct termios *oldtio, struct termios *newtio)
{
  *fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK | O_ASYNC);
  if (*fd < 0) {
    perror(MODEMDEVICE);
    exit(-1);
  } else printf("Opened device %s.\n", MODEMDEVICE);
  
  /* Set Baud, bits, parity, stop bit:
     (9600, 8 bits, no parity, 1 stop bit) */
  tcgetattr(*fd, oldtio);  /* Save old serial port settings. */
  memset(newtio, 0, sizeof(*newtio));  /* Clear newtio. */
  newtio->c_cflag = INITBN | CS8 | CLOCAL | CREAD;
  newtio->c_iflag = IGNPAR;
  newtio->c_oflag = 0;  /* Raw output, which I think is the best bet
			   for getting the camera to work. */
  newtio->c_lflag = 0;  /* Non-canonical read; i.e., no processing
			   of input. */
  newtio->c_cc[VTIME] = 0;
  newtio->c_cc[VMIN] = 1;  /* Only require one character to read 
			      (but below I will read all that are
			      available, up to the 255-character
			      buffer.) */

  /* Flush serial port and apply new settings: */
  tcflush(*fd, TCIFLUSH);
  tcsetattr(*fd, TCSANOW, newtio);  
}


/* -------------------------------------------------------------------
   Close serial connection with cpu. */
void closecpu(int *fd, struct termios *oldtio)
{
  tcsetattr(*fd, TCSANOW, oldtio);  /* Restore serial port
				       settings. */
  int res = close(*fd);
  if (res < 0) {
    perror(MODEMDEVICE);
    exit(-1);
  } else printf("Closed device %s.\n", MODEMDEVICE);
}


/* -------------------------------------------------------------------
   Set up stdin as a non-blocking device (so I can read
   charactes as they come, rather than waiting for an Enter.) */
void openstdin(int *fd, struct termios *oldtio, struct termios *newtio)
{
  *fd = fileno(stdin);

  tcgetattr(*fd, oldtio);  /* Save old stdin terminal settings: */
  memcpy(newtio, oldtio, sizeof(*oldtio));  /* Start with current settings. */

  /* Set flags for noncanonical input, no buffering: */
  newtio->c_lflag = 0;  /* No input processing. */
  newtio->c_cc[VTIME] = 0;
  newtio->c_cc[VMIN] = 1;

  /* Flush input and apply new settings: */
  tcflush(*fd, TCIFLUSH);
  tcsetattr(*fd, TCSANOW, newtio);
}


/* -------------------------------------------------------------------
   Restore settings for stdin: */
void closestdin(int *fd, struct termios *oldtio)
{
  tcsetattr(*fd, TCSANOW, oldtio);
}


/* -------------------------------------------------------------------
   Lower level utilities:
   ------------------------------------------------------------------- */


/* -------------------------------------------------------------------
   outstr:  For debugging.
   Write string to stdout as a column of hex bytes. */
void outstr(unsigned char *s, int len)
{
  int i;
  for (i = 0; i < len; i++)
    printf("%4d    %2.2xH    %c\n", i, s[i], s[i]);
}


/* -------------------------------------------------------------------
   getint:  The cpu transmits integers as middle-endian (i.e., low
   byte first, high bit second). That's inconvenient, so this routine
   converts two response bytes to an integer. */
int getint(unsigned char *data)
{
  return data[0] + data[1] * 0x0100;
}


/* -------------------------------------------------------------------
   convint:  And do the reverse, for building commands. */
void convint(unsigned char *result, unsigned int data)
{
  result[0] = data & 0xff;
  result[1] = (data >> 8) & 0xff;
}


/* -------------------------------------------------------------------
   convlong:  Also convert long integers. */
void convlong(unsigned char *result, unsigned long int data)
{
  result[0] = data & 0xff;
  result[1] = (data >> 8) & 0xff;
  result[2] = (data >> 16) & 0xff;
  result[3] = (data >> 24) & 0xff;
}


/* -------------------------------------------------------------------
   bcdfour:  Convert four-digit BCD number to double.  Use bitwise
   shift with bitwise AND to break out individual hex characters. */
double bcdfour(unsigned char *s)
{
  int i, j;
  double p, num;

  p = 0.01;
  num = 0;
  for (j = 0; j < 2; j++)
    for (i = 0; i < 2; i++) {
      num += ((s[j] >> 4*i) & 0x0f) * p;
      p *= 10;
    }

  return num;
}


/* -------------------------------------------------------------------
   bcdeight:  Convert eight-digit BCD number to double. */
double bcdeight(unsigned char *s)
{
  int i, j;
  double p, num;

  p = 0.01;
  num = 0;
  for (j = 0; j < 4; j++)
    for (i = 0; i < 2; i++) {
      num += ((s[j] >> 4*i) & 0x0f) * p;
      p *= 10;
    }

  return num;
}


/* -------------------------------------------------------------------
   getchecksum:  Compute checksum. */
void getchecksum(unsigned char *checksum,
		 unsigned char *pak, int paklen)
{
  unsigned int cs = 0;
  int i;
  for (i = 0; i < paklen; i++)
    cs += pak[i];
  convint(checksum, cs);
}


/* -------------------------------------------------------------------
   checksum:  Compute checksum on (buf minus the last two bytes), then
   compare with the given checksum.  Return TRUE if they match, FALSE
   if they don't match. */
int checksum(unsigned char *buf, int len)
{
  unsigned char cs[2];
  getchecksum(cs, buf, len-2);
  if (cs[0] == buf[len-2] && cs[1] == buf[len-1])
    return TRUE;
  else
    return FALSE;
}


/* -------------------------------------------------------------------
   printinstructions:  Print a list of things that can be done.
   Initially I think this will just be (x) Exit, (i) Take image, (s)
   Toggle shutter. Eventually add things like set temperature. */
void printinstructions(void)
{
  printf("\n");
  printf("+----------------------------------------+\n");
  printf("| OPTIONS:                               |\n");
  printf("|                                        |\n");
  printf("| i - Take image                         |\n");
  printf("| d - Take dark image                    |\n");
  printf("| s - Toggle shutter                     |\n");
  printf("| b - Switch to higher baud rate         |\n");
  printf("| x - Exit                               |\n");
  printf("+----------------------------------------+\n\n");
}


/* -------------------------------------------------------------------
   plotimage:  Pipe some commands to gnuplot to plot the image. */
void plotimage(void)
{
  FILE *pipe = popen("gnuplot -persist", "w");

  fprintf(pipe, "unset border\n");
  fprintf(pipe, "unset tics\n");
  fprintf(pipe, "unset colorbox\n");
  fprintf(pipe, "set palette gray\n");
  fprintf(pipe, "plot '%s' matrix with image\n", IMGFILE);

  pclose(pipe);
}


/* -------------------------------------------------------------------
   Then a different parse for each response I could be dealing with:
   ------------------------------------------------------------------- */


/* -------------------------------------------------------------------
   Parse get_cpu_info:  The manual has a
   description of the response data.  Parse this and display in a
   human-readable format. */
void processcpuinfo(unsigned char *data, int len)
{
  /* Camera name: */
  char name[32];
  memcpy(name, &data[10], sizeof(name));
  printf("\n%s\n", name);
  
  printf("Firmware version:  %.2f\n", bcdfour(&data[8]));

  /* Some other properties: */
  printf("Shutter (Dark Vane):  ");
  if (getint(&data[42]) == 1) printf("YES\n");
  else printf("NO\n");

  printf("Need to initialize with set_head_offset:  ");
  if (getint(&data[44]) == 1) printf("YES\n");
  else printf("NO\n");

  printf("Programmable double-correlated sampling (DCS):  ");
  if (getint(&data[46]) == 1) printf("YES\n");
  else printf("NO\n");

  printf("Programmable DCS restore:  ");
  if (getint(&data[48]) == 1) printf("YES\n");
  else printf("NO\n");

  printf("Closed-loop temperature regulation:  ");
  if (getint(&data[50]) == 1) printf("YES\n");
  else printf("NO\n");

  printf("Maximum TE-cooler drive value:  %d\n", getint(&data[52]));

  printf("Image dimensions:  %d x %d\n",
	 getint(&data[54]), getint(&data[56]));

  /* Readout modes: */
  int i;
  int nmodes = getint(&data[58]);
  printf("\nNumber of readout modes:  %d\n\n", getint(&data[58]));
  for (i = 0; i < nmodes; i++)
    printf("Mode %d:  %d x %d px, Gain:  %.2f, Pixel Dims (um):  %.1f x %.1f\n",
	   getint(&data[60+16*i]), getint(&data[62+16*i]), getint(&data[64+16*i]),
	   bcdfour(&data[66+16*i]), bcdeight(&data[68+16*i]), bcdeight(&data[72+16*i]));
}


/* -------------------------------------------------------------------
   Communications functions:
   ------------------------------------------------------------------- */


/* -------------------------------------------------------------------
   buildpacket:  Put the command packet together, calculate checksum,
   and append to the end of the packet. */
void buildpacket(unsigned char *pak, int *len,
		 unsigned char cmd, unsigned char *data,
		 int datalen)
{
  int i;
  unsigned char dl[2];
  unsigned char cs[2];
  
  /* Start from scratch: */
  memset(pak, 0, sizeof(pak));
  *len = 0;
  
  pak[0] = 0xa5;
  pak[1] = cmd;
  *len += 2;

  /* Data length: */
  convint(dl, datalen);
  pak[2] = dl[0];
  pak[3] = dl[1];
  *len += 2;

  /* Data: */
  for (i = 0; i < datalen; i++) {
    pak[i + 4] = data[i];
  }
  *len += datalen;

  /* Checksum: */
  getchecksum(cs, pak, *len);
  pak[4 + datalen] = cs[0];
  pak[5 + datalen] = cs[1];
  *len += 2;
}


/* -------------------------------------------------------------------
   listen:  This is the core function for reading command packets from
   the cpu.  It works as follows:

   Listen to the serial connection with the cpu.  Wait for
   ACK, NAK, CAN, or 0xa5; ignore everything else.  If ACK, NAK, or
   CAN, return 1 and the unsigned char for ACK, NAK, or CAN.  If 0xa5,
   wait for the rest of the packet, perform a checksum, then return
   the packet length.

   Error codes:
   If timeout before receiving anything:  return -1.
   If bad checksum, or timeout before reading entire command:
   return -2.

   Whatever has been received, even if incomplete, will be stored in
   the string pointed to by pak. */
int listen(int fd, unsigned char* pak)
{
  unsigned char buf[BUFLEN];
  fd_set readfs;
  struct timeval tout;
  int ret, res, STOP, TIMEOUT;
  int maxfd = fd + 1;
  int len = 0;

  /* Wait for the first meaningful character: */
  STOP = FALSE;
  TIMEOUT = FALSE;
  FD_ZERO(&readfs);
  tout.tv_sec = 2;
  tout.tv_usec = 560000;  /* Timeout after 2.56 s. */
  while (STOP == FALSE) {
    FD_SET(fd, &readfs);

    /* Block until we have some input
       from cpu, but only for TIMEOUT. */
    ret = select(maxfd, &readfs, NULL, NULL, &tout);

    /* Did we get a character? ... */
    if (FD_ISSET(fd, &readfs)) {
      res = read(fd, buf, BUFLEN);
      if (res > 0) {
	/* We've received a character!
	   is it something meaningful? */
	if (buf[0] == 0x06 || buf[0] == 0x15 ||
	    buf[0] == 0x18 || buf[0] == 0xa5) {
	  STOP = TRUE;
	  memcpy(pak, buf, res);
	  len += res;
	}
      }
    }
    
    /* ... Or did we get a timeout? */
    if (ret == 0) {
      STOP = TRUE;
      TIMEOUT = TRUE;
    }
  }

  /* Handle errors / timeout reading first byte: */
  if (TIMEOUT == TRUE) return -1;
  if (pak[0] == 0x06 || pak[0] == 0x15 || pak[0] == 0x18) return 1;

  STOP = FALSE;
  /* If I already have a complete command, I can just stop here: */
  if (len >= 6 && (len - 6) == getint(&pak[2]))
    STOP = TRUE;

  /* Otherwise, keep reading until we get a timeout: */
  TIMEOUT = FALSE;
  FD_ZERO(&readfs);
  tout.tv_sec = 2;
  tout.tv_usec = 560000;
  while (STOP == FALSE) {
    FD_SET(fd, &readfs);

    /* Block until we have some input
       from cpu, but only for TIMEOUT. */
    ret = select(maxfd, &readfs, NULL, NULL, &tout);

    /* Did we get some characters? ... */
    if (FD_ISSET(fd, &readfs)) {
      res = read(fd, buf, BUFLEN);
      if (res > 0) {
	memcpy(&pak[len], buf, res);
	len += res;
      }
    }

    /* ... Or did we get a timeout? */
    if (ret == 0) {
      STOP = TRUE;
      TIMEOUT = TRUE;
    }

    /* If I already have a complete command (meaning six bytes
       plus the expected data length), I can stop: */
    if (len >= 6 && (len - 6) == getint(&pak[2])) STOP = TRUE;
  }
  
  /* Check the checksum: */
  if (checksum(pak, len) == TRUE)
    return len;
  else
    return -2;
}


/* -------------------------------------------------------------------
   Some routines for the specific commands that I need to send:
   ------------------------------------------------------------------- */


/* -------------------------------------------------------------------
   checkrom:  Check connection by sending get_rom_version.  Write the
   returned firmware version to stdout. */
int checkrom(int fd)
{
  unsigned char pak[BUFLEN];
  unsigned char buf[BUFLEN];
  int len, res;

  buildpacket(pak, &len, 0x19, NULL, 0);

  /* Send packet to cpu: */
  write(fd, pak, len);
  
  /* Listen for response: */
  res = listen(fd, buf);

  if (res > 1) {
    printf("Connected to CPU, firmware version %.2f.\n",
	   bcdfour(&buf[4]));
    return TRUE;
  } else if (res == -2)
    printf("Bad checksum.\n");
  else
    printf("No response from CPU.  Check connection?\n");

  return FALSE;
}


/* -------------------------------------------------------------------
   getcpuinfo:  Print out stats about the camera. */
void getcpuinfo(int fd)
{
  unsigned char pak[BUFLEN];
  unsigned char buf[BUFLEN];
  int len, res;

  buildpacket(pak, &len, 0x25, NULL, 0);

  /* Send packet to cpu: */
  write(fd, pak, len);

  /* Listen for response: */
  res = listen(fd, buf);

  /* Less error checking than checkrom; if we got this far,
     there should be a connection to the CPU.  Just let the
     user know if it fails, and he can decide what to do. */
  if (res > 1)
    processcpuinfo(buf, res);
  else
    printf("No response from CPU to get_cpu_info.\n");
}


/* -------------------------------------------------------------------
   uncompressline:  Uncompress one image line.  See manual for
   compression algorithm. */
void uncompressline(unsigned char *raw, unsigned char *buf)
{
  int n, m;
  int base, delta;

  /* Copy first six bytes of packet. */
  memcpy(buf, raw, 6);

  /* Uncompress data: */
  base = raw[6]*0x0100 + raw[7];
  convint(&buf[6], base);
  m = 0;
  for (n = 0; n < 500; n += 2) {
    switch ((raw[m+8] >> 6) & 3) {
    case 3:
      base = (raw[m+8]*0x0100 + raw[m+9]) - 0xc000;
      base *= 4;
      convint(&buf[n+8], base);
      m += 2;
      break;
    case 2:
      delta = (raw[m+8]*0x0100 + raw[m+9]) - 0x8000;
      if (delta > 8191) delta -= 16384;
      base += delta;
      if (base > 65535) base -= 65536;
      convint(&buf[n+8], base);
      m += 2;
      break;
    default:
      delta = raw[m+8];
      if (delta > 63) delta -= 128;
      base += delta;
      if (base > 65535) base -= 65536;
      convint(&buf[n+8], base);
      m += 1;
      break;
    }
  }
  /* Don't bother copying checksum. */
}


/* -------------------------------------------------------------------
   readline:  Read one image line.  Return FALSE if it fails, so the
   calling function can try this line again. */
int  readline(int n, int fd, unsigned char *buf)
{
  unsigned char pak[BUFLEN];
  unsigned char raw[BUFLEN];
  int len, res;

  /* Parameters: */
  unsigned char p[8];
  convint(&p[0], 1);  /* buffer to read from */
  convint(&p[2], n);  /* line to read */
  convint(&p[4], 0);  /* leftmost pixel */
  convint(&p[6], 250);  /* number of pixels */
  
  buildpacket(pak, &len, 0x07, p, 8);
  /* buildpacket(pak, &len, 0x1f, p, 8); */

  /* Send packet. */
  write(fd, pak, len);
  
  /* Listen for line. */
  res = listen(fd, raw);

  /* If line received with no errors, write to file. */
  if (res > 1) {
    uncompressline(raw, buf);
    /* memcpy(buf, raw, res); */
    return TRUE;
  } else return FALSE;  /* Otherwise, decrement n to try this line again. */      
}


/* -------------------------------------------------------------------
   checkidle:  Poll the cpu to see if Status is Idle (0) */
int checkidle(int fd)
{
  unsigned char pak[BUFLEN];
  unsigned char buf[BUFLEN];
  int len, res;

  /* Parameters: */
  unsigned char p[2];
  p[0] = 0x01;
  p[1] = 0x00;

  buildpacket(pak, &len, 0x05, p, 2);
  write(fd, pak, len);
  res = listen(fd, buf);

  if (res > 1 && getint(&buf[4]) == 1 && getint(&buf[6]) == 0)
    return TRUE;
  else
    return FALSE;
}


/* -------------------------------------------------------------------
   takeimage:  Take an image, write the pixel array to IMAGEFILE.  For
   now, use default settings.  Later, allow changes. */
int takeimage(int fd)
{
  unsigned char pak[BUFLEN];
  unsigned char buf[BUFLEN];
  int len, res;
  int DONE = FALSE;

  /* Parameters: */
  unsigned char p[28];
  convint(&p[0], 200);  /* exposure time, 100ths of a second */
  convint(&p[2], 0);
  convint(&p[4], 0);  /* top line to read out */
  convint(&p[6], 121);  /* number of lines to read out */
  convint(&p[8], 0);  /* leftmost pixel  to read out*/
  convint(&p[10], 250);  /* number of pixels to read out */
  convint(&p[12], 0);  /* bool:  double-correlated readout? */
  convint(&p[14], 0);  /* bool:  do DC restore on readout? */
  convint(&p[16], 0);  /* antiblooming gate state */
  convint(&p[18], 6000);  /* antiblooming period */
  convint(&p[20], 1);  /* destination buffer for readout */
  convint(&p[22], 1);  /* subtract dark buffer before saving */
  convint(&p[24], 3);  /* readout mode */
  convint(&p[26], 2);  /* (3 options):  open shutter? */
  
  buildpacket(pak, &len, 0x01, p, 28);

  /* Send packet: */
  write(fd, pak, len);

  /* Listen for ACK: */
  res = listen(fd, buf);
  if (!(res == 1 && buf[0] == 0x06)) {
    printf("Problem taking image.\n");
    return FALSE;
  } else
    printf("Taking image ...\n");

  /* Loop, poll cpu every 0.5 seconds
     to see if the exposure is finished. */
  struct timespec wait;
  wait.tv_sec = 0;
  wait.tv_nsec = 370000000;
  while (!(checkidle(fd) == TRUE)) {
    nanosleep(wait, NULL);
  }
  printf("Exposure completed!  Ready for readout.\n");

  /* Open text file. */
  FILE *fid;
  fid = fopen(IMGFILE, "w+");

  /* Read out image lines and write to file. */
  int n, i;
  printf("Reading image:\n");
  printf("[                                                            ]\r");
  printf("["); fflush(stdout);
  for (n = 0; n < 120; n++) {
    if (readline(n, fd, buf) == FALSE) n--;
    else {
      if ((n & 1) == 0) {
	printf("-");
	fflush(stdout);
      }
      for (i = 0; i < 250; i++)
	fprintf(fid, "%d ", getint(&buf[6+2*i]));
      fprintf(fid, "\n");
    }
  }
  
  fclose(fid);

  return TRUE;
}


/* -------------------------------------------------------------------
   takedarkimage:  Take a dark image, and store in the dark buffer. */
int takedarkimage(int fd)
{
  unsigned char pak[BUFLEN];
  unsigned char buf[BUFLEN];
  int len, res;
  int DONE = FALSE;

  /* Parameters: */
  unsigned char p[28];
  convint(&p[0], 200);  /* exposure time, 100ths of a second */
  convint(&p[2], 0);
  convint(&p[4], 0);  /* top line to read out */
  convint(&p[6], 121);  /* number of lines to read out */
  convint(&p[8], 0);  /* leftmost pixel  to read out*/
  convint(&p[10], 250);  /* number of pixels to read out */
  convint(&p[12], 0);  /* bool:  double-correlated readout? */
  convint(&p[14], 0);  /* bool:  do DC restore on readout? */
  convint(&p[16], 0);  /* antiblooming gate state */
  convint(&p[18], 6000);  /* antiblooming period */
  convint(&p[20], 0);  /* destination buffer for readout */
  convint(&p[22], 0);  /* subtract dark buffer before saving */
  convint(&p[24], 3);  /* readout mode */
  convint(&p[26], 0);  /* (3 options):  open shutter? */
  
  buildpacket(pak, &len, 0x01, p, 28);

  /* Send packet: */
  write(fd, pak, len);

  /* Listen for ACK: */
  res = listen(fd, buf);
  if (!(res == 1 && buf[0] == 0x06)) {
    printf("Problem taking image.\n");
    return FALSE;
  } else
    printf("Taking image ...\n");

  /* Loop, poll cpu every 0.5 seconds
     to see if the exposure is finished. */
  struct timespec wait;
  wait.tv_sec = 0;
  wait.tv_nsec = 370000000;
  while (!(checkidle(fd) == TRUE)) {
    nanosleep(wait, NULL);
  }
  printf("Exposure completed!  Image stored in dark buffer.\n");

  return TRUE;
}


/* -------------------------------------------------------------------
   toggleshutter:  Open or close the shutter. */
void toggleshutter(int fd, int *shutter)
{
  unsigned char pak[BUFLEN];
  unsigned char buf[BUFLEN];
  int len, res;

  /* Parameters: */
  unsigned char p[2];
  p[1] = 0x00;
  if (*shutter == OPEN) {
    p[0] = 0x01;
    *shutter = CLOSED;
    printf("Closing shutter.\n");
  } else if (*shutter == CLOSED) {
    p[0] = 0X00;
    *shutter = OPEN;
    printf("Opening shutter.\n");
  }

  buildpacket(pak, &len, 0x04, p, 2);
  write(fd, pak, len);

  res = listen(fd, buf);

  if (!(res == 1 && buf[0] == 0x06))
    printf("ACK not received.  Check connection.\n");
}


/* -------------------------------------------------------------------
   sethoffset:  Set head offset. */
void sethoffset(int fd, int hoffset)
{
  unsigned char pak[BUFLEN];
  unsigned char buf[BUFLEN];
  int len, res;

  /* Parameters: */
  unsigned char p[2];
  convint(p, hoffset);

  buildpacket(pak, &len, 0x0f, p, 2);
  write(fd, pak, len);
  res = listen(fd, buf);

  if (!(res == 1 && buf[0] == 0x06))
    printf("ACK not received.  Check connection.\n");
}


/* -------------------------------------------------------------------
   checkhoffset:  Try a value of 175 for head offset and return TRUE
   if the returned video parameter is between 1000 and 10000. */
int checkhoffset(int fd, int hoffset)
{
  unsigned char pak[BUFLEN];
  unsigned char buf[BUFLEN];
  int len, res;
  int video;

  /* Parameters: */
  unsigned char p[4];
  p[0] = 0x01;  /* set enable_dcs to TRUE. */
  p[1] = 0x00;
  convint(&p[2], hoffset);  /* head_offset */

  buildpacket(pak, &len, 0x12, p, 4);
  
  /* Send packet to cpu: */
  write(fd, pak, len);

  /* Listen for response: */
  res = listen(fd, buf);

  if (res > 1)
    video = getint(&buf[4]);
  else {
    printf("No response from CPU to read_blank_video.\n");
    return FALSE;
  }
  
  printf("CCD black level:  %d\n", video);
  if (video >= 1000 && video <= 10000)
    return TRUE;
  else return FALSE;
}


/* -------------------------------------------------------------------
   changebaud:  Switch to a higher baud rate.  Afterwards, send a
   get_rom_version and then one get_line.  If all of this works
   without error, keep the changed baud rate and return TRUE.
   Otherwise, return FALSE. */
int changebaud(int fd, long int speed, speed_t speedname)
{
  unsigned char pak[BUFLEN];
  unsigned char buf[BUFLEN];
  int len, res;

  /* Parameters: */
  unsigned char p[4];
  convlong(p, speed);

  buildpacket(pak, &len, 0x1a, p, 4);

  /* Send packet: */
  printf("Trying %ld baud.\n", speed);
  write(fd, pak, len);

  /* Listen for ACK: */
  res = listen(fd, buf);
  if (!(res == 1 && buf[0] == 0x06)) {
    printf("Problem sending set_com_baud.\n");
    return FALSE;
  }

  /* Okay, the cpu got the set_com_baud command.
     Now change MY baud rate and try a get_rom_version. */
  struct termios tio;
  tcgetattr(fd, &tio);
  cfsetspeed(&tio, speedname);
  tcsetattr(fd, TCSANOW, &tio);
  if (!(checkrom(fd) == TRUE)) {
    printf("Bad response at new baud rate.\n");
    cfsetspeed(&tio, INITBN);
    tcsetattr(fd, TCSANOW, &tio);
    return FALSE;
  }

  /* Okay, that went fine.  Now try a get_line. */
  if (!(readline(0, fd, buf))) {
    printf("Problem reading line\n");
    cfsetspeed(&tio, INITBN);
    tcsetattr(fd, TCSANOW, &tio);
    return FALSE;
  }

  return TRUE;
}


/* -------------------------------------------------------------------
   Main 
   ------------------------------------------------------------------- */


int main(void)
{
  /* Initialize empty buffer for input from
     cpu. */
  unsigned char buf[BUFLEN];
  int len = 0;

  /* Initialize file descriptors and termios structures
     for handling the connection with the cpu, and stdin. */
  int fdcpu, fdstdin;
  struct termios oldcputio, newcputio,
    oldstdintio, newstdintio;

  int STOP = FALSE;
  int SHUTTER = OPEN;

  /* Open serial connection with cpu: */
  opencpu(&fdcpu, &oldcputio, &newcputio);

  /* Setup stdin as non-block device: */
  openstdin(&fdstdin, &oldstdintio, &newstdintio);

  /* Check connection and get firmware version: */
  if (!checkrom(fdcpu)) {
    closestdin(&fdstdin, &oldstdintio);
    closecpu(&fdcpu, &oldcputio);
    return -1;
  }

  /* Print out stats about the camera: */
  getcpuinfo(fdcpu);

  /* Check head_offset: */
  printf("\n");
  if (SHUTTER == OPEN) toggleshutter(fdcpu, &SHUTTER);
  printf("Trying head_offset of %d:\n", HEADOFFSET);
  if (checkhoffset(fdcpu, HEADOFFSET) == TRUE)
    printf("That's fine.  It should be between 1000 and 10,000.\n");
  else {
    printf("NOT within range.  Shutting down.\n");
    closestdin(&fdstdin, &oldstdintio);
    closecpu(&fdcpu, &oldcputio);
    return -1;
  }
  toggleshutter(fdcpu, &SHUTTER);

  /* Set head offset: */
  printf("\n");
  sethoffset(fdcpu, HEADOFFSET);
  printf("Set head offset to %d.\n", HEADOFFSET);

  /* Now wait for the user to press a key: */
  fd_set readfs;
  int maxfd = fdstdin + 1;
  FD_ZERO(&readfs);
  while (STOP == FALSE) {

    /* Print user instructions: */
    printinstructions();

    FD_SET(fdstdin, &readfs);

    /* Block until we have some input */
    select(maxfd, &readfs, NULL, NULL, NULL);
    if (FD_ISSET(fdstdin, &readfs)) {
      len = read(fdstdin, buf, 1);
      if (len > 0) {
	if (buf[0] == 'x')
	  STOP = TRUE;
	if (buf[0] == 's')
	  toggleshutter(fdcpu, &SHUTTER);
	if (buf[0] == 'i')
	  if (takeimage(fdcpu) == TRUE) {
	    printf("\nWrote pixel array to %s.\n", IMGFILE);
	    plotimage();
	  } else
	    printf("\nNo image taken.\n");
	if (buf[0] == 'b')
	  if (changebaud(fdcpu, HIGHBAUD, HIGHBN) == TRUE)
	    printf("Baud rate now %ld.\n", (long int)HIGHBAUD);
	  else
	    printf("Higher baud rate failed.\n");
	if (buf[0] == 'd') {
	  if (takedarkimage(fdcpu) == TRUE)
	    printf("Stored dark image in buffer.\n");
	  else
	    printf("Dark image failed.\n");
	}
      }
    }
  }

  /* Open the shutter. */
  if (SHUTTER == CLOSED) toggleshutter(fdcpu, &SHUTTER);

  /* Reset baud rate: */
  changebaud(fdcpu, INITBAUD, INITBN);

  /* Reset stdin. */
  closestdin(&fdstdin, &oldstdintio);

  /* Close serial connection with cpu. */
  closecpu(&fdcpu, &oldcputio);

  return 0;
}
