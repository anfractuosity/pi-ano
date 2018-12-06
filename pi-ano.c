/*
    Pi-ano 

    Uses:

    * https://www.raspberrypi.org/forums/viewtopic.php?f=29&t=52393 - to disable interrupts and for gpio setup

    * https://elinux.org/RPi_GPIO_Code_Samples#Direct_register_access - for IO macros

    * http://portaudio.com/docs/v19-doxydocs/paex__sine_8c_source.html - for generating tones
*/

#include <stdio.h>
#include <math.h>
#include <portaudio.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdio.h>		
#include <fcntl.h>	
#include <sys/mman.h>		
#include <unistd.h>

#define NUM_SECONDS   (5)
#define SAMPLE_RATE   (44100)
#define FRAMES_PER_BUFFER  (64)

#ifndef M_PI
#define M_PI  (3.14159265)
#endif

#define TABLE_SIZE   (200)
typedef struct {
	float sine[TABLE_SIZE];
	int left_phase;
	int right_phase;
	char message[20];
} paTestData;

int setup(void);
int interrupts(int flag);
int fdgpio;
int mem_fd;
void *gpio_map;

// I/O access
volatile unsigned *gpio;

#define GPIO_BASE  0x20200000
#define TIMER_BASE 0x20003000
#define INT_BASE 0x2000B000

volatile unsigned *gpio, *gpset, *gpclr, *gpin, *timer, *intrupt;

/******************** GPIO read/write *******************/
  // outputs  CLK = GPIO 15 = connector pin 10
  //          DAT = GPIO 14 = connector pin 8
  // code example
  //    CLKHI;
#define CLKHI *gpset = (1 << 15)	// GPIO 15
#define CLKLO *gpclr = (1 << 15)
#define DATHI *gpset = (1 << 14)	// GPIO 14
#define DATLO *gpclr = (1 << 14)
  // inputs   P3  = GPIO 3 = connector pin 5 (Rev 2 board)
  //          P2  = GPIO 2 = connector pin 3 (Rev 2 board)
  //          ESC = GPIO 18 = connector pin 12
  // code examples
  //   if(P2IN == 0)
  //   if(P2IN != 0)
  //   n = P2INBIT;  //  0 or 1
#define ESCIN (*gpin & (1 << 18))	// GPIO 18
#define P2IN (*gpin & (1 << 2))	// GPIO 2
#define P3IN (*gpin & (1 << 3)	// GPIO 3
#define P2INBIT ((*gpin >> 2) & 1)	// GPIO 2
#define P3INBIT ((*gpin >> 3) & 1)	// GPIO 3
/******************* END GPIO ****************/
#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)	// sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10)	// clears bits which are 1 ignores bits which are 0

#define GET_GPIO(g) (*(gpio+13)&(1<<g))	// 0 if LOW, (1<<g) if HIGH

#define GPIO_PULL *(gpio+37)	// Pull up/pull down
#define GPIO_PULLCLK0 *(gpio+38)	// Pull up/pull down clock

PaStreamParameters outputParameters;
PaStream *stream;
PaError err;
paTestData data;
int i;

int setup() {
	int memfd;
	unsigned int timend;
	void *gpio_map, *timer_map, *int_map;

	memfd = open("/dev/mem", O_RDWR | O_SYNC);
	if (memfd < 0) {
		printf("Mem open error\n");
		return (0);
	}

	gpio_map = mmap(NULL, 4096, PROT_READ | PROT_WRITE,
			MAP_SHARED, memfd, GPIO_BASE);

	timer_map = mmap(NULL, 4096, PROT_READ | PROT_WRITE,
			 MAP_SHARED, memfd, TIMER_BASE);

	int_map = mmap(NULL, 4096, PROT_READ | PROT_WRITE,
		       MAP_SHARED, memfd, INT_BASE);

	close(memfd);

	if (gpio_map == MAP_FAILED ||
	    timer_map == MAP_FAILED || int_map == MAP_FAILED) {
		printf("Map failed\n");
		return (0);
	}
	// interrupt pointer
	intrupt = (volatile unsigned *)int_map;
	// timer pointer
	timer = (volatile unsigned *)timer_map;
	++timer;		// timer lo 4 bytes
	// timer hi 4 bytes available via *(timer+1)

	// GPIO pointers
	gpio = (volatile unsigned *)gpio_map;
	gpset = gpio + 7;	// set bit register offset 28
	gpclr = gpio + 10;	// clr bit register
	gpin = gpio + 13;	// read all bits register

	// setup  GPIO 2/3 = inputs    have pull ups on board
	//        control reg = gpio + 0 = pin/10
	//        GPIO 2 shift 3 bits by 6 = (pin rem 10) * 3
	//        GPIO 3 shift 3 bits by 9 = (pin rem 10) * 3

	*gpio &= ~(7 << 6);	// GPIO 2  3 bits = 000 input
	*gpio &= ~(7 << 9);	// GPIO 3  3 bits = 000 input

	// setup GPIO 18 = input
	*(gpio + 1) &= ~(7 << 24);	// GPIO 18 input
	// enable pull up on GPIO 18
	*(gpio + 37) = 2;	// PUD = 2  pull up
	//     = 0  disable pull up/down
	//     = 1  pull down
	timend = *timer + 2;	// 2us delay
	while ((((*timer) - timend) & 0x80000000) != 0) ;
	*(gpio + 38) = (1 << 18);	// PUDCLK bit set clocks PUD=2 to GPIO 18
	timend = *timer + 2;	// 2us delay
	while ((((*timer) - timend) & 0x80000000) != 0) ;
	*(gpio + 37) = 0;	// zero PUD
	*(gpio + 38) = 0;	// zero PUDCLK
	// finished pull up enable

	//      GPIO 14/15 = outputs
	//      control reg = gpio + 1 = pin/10
	//      GPIO 14 shift 3 bits by 12 = (pin rem 10) * 3
	//      GPIO 15 shift 3 bits by 15 = (pin rem 10) * 3

	*(gpio + 1) &= ~(7 << 12);	// GPIO 14 zero 3 bits
	*(gpio + 1) |= (1 << 12);	// 3 bits = 001 output

	*(gpio + 1) &= ~(7 << 15);	// GPIO 15 zero 3 bits
	*(gpio + 1) |= (1 << 15);	// 3 bits = 001 output

	return (1);
}

static int patestCallback(const void *inputBuffer, void *outputBuffer,
			  unsigned long framesPerBuffer,
			  const PaStreamCallbackTimeInfo * timeInfo,
			  PaStreamCallbackFlags statusFlags, void *userData) {
	paTestData *data = (paTestData *) userData;
	float *out = (float *)outputBuffer;
	unsigned long i;

	(void)timeInfo;		/* Prevent unused variable warnings. */
	(void)statusFlags;
	(void)inputBuffer;

	for (i = 0; i < framesPerBuffer; i++) {
		*out++ = data->sine[data->left_phase];	/* left */
		*out++ = data->sine[data->right_phase];	/* right */
		data->left_phase += 1;
		if (data->left_phase >= TABLE_SIZE)
			data->left_phase -= TABLE_SIZE;
		data->right_phase += 3;	/* higher pitch so we can distinguish left and right. */
		if (data->right_phase >= TABLE_SIZE)
			data->right_phase -= TABLE_SIZE;
	}

	return paContinue;
}

/*
 * This routine is called by portaudio when playback is done.
 */
static void StreamFinished(void *userData) {
	paTestData *data = (paTestData *) userData;
	printf("Stream Completed: %s\n", data->message);
}

main() {

	int tone;
	int n, getout;
	unsigned int timend;

	int lcount = 0;

	printf("PortAudio Test: output sine wave. SR = %d, BufSize = %d\n",
	       SAMPLE_RATE, FRAMES_PER_BUFFER);

	/* initialise sinusoidal wavetable */
	for (i = 0; i < TABLE_SIZE; i++) {
		data.sine[i] =
		    (float)sin(((double)i / (double)TABLE_SIZE) * M_PI * 2.);
	}
	data.left_phase = data.right_phase = 0;

	err = Pa_Initialize();
	if (err != paNoError)
		goto error;

	outputParameters.device = Pa_GetDefaultOutputDevice();	/* default output device */
	if (outputParameters.device == paNoDevice) {
		fprintf(stderr, "Error: No default output device.\n");
		goto error;
	}
	outputParameters.channelCount = 2;	/* stereo output */
	outputParameters.sampleFormat = paFloat32;	/* 32 bit floating point output */
	outputParameters.suggestedLatency =
	    Pa_GetDeviceInfo(outputParameters.device)->defaultLowOutputLatency;
	outputParameters.hostApiSpecificStreamInfo = NULL;

	err = Pa_OpenStream(&stream, NULL,	/* no input */
			    &outputParameters, SAMPLE_RATE, FRAMES_PER_BUFFER, paClipOff,	/* we won't output out of range samples so don't bother clipping them */
			    patestCallback, &data);
	if (err != paNoError)
		goto error;

	sprintf(data.message, "No Message");
	err = Pa_SetStreamFinishedCallback(stream, &StreamFinished);
	if (err != paNoError)
		goto error;

	setup();

	sleep(1);

	while (1) {
		interrupts(0);

		INP_GPIO(17);
		GPIO_PULL = 2;
		GPIO_PULLCLK0 = 1 << 17;

		long read;
		long z = 0;
		while (1) {
			read = gpio[13] & (1 << 17);
			z++;
			if (read > 0)
				break;
		}

		OUT_GPIO(17);
		interrupts(1);

		if (z > 9) {
			if (tone != 1) {
				err = Pa_StartStream(stream);
				if (err != paNoError)
					goto error;
			}
			tone = 1;
			lcount = 0;
		} else {
			lcount++;
			if (tone == 1 && lcount > 60) {
				err = Pa_StopStream(stream);
				if (err != paNoError)
					goto error;
				tone = 0;
				lcount = 0;
			}
		}

	}

	return err;
 error:
	Pa_Terminate();
	fprintf(stderr, "An error occured while using the portaudio stream\n");
	fprintf(stderr, "Error number: %d\n", err);
	fprintf(stderr, "Error message: %s\n", Pa_GetErrorText(err));
	return err;

}

int interrupts(int flag) {
	static unsigned int sav132 = 0;
	static unsigned int sav133 = 0;
	static unsigned int sav134 = 0;

	if (flag == 0)		// disable
	{
		if (sav132 != 0) {
			// Interrupts already disabled so avoid printf
			return (0);
		}

		if ((*(intrupt + 128) | *(intrupt + 129) | *(intrupt + 130)) !=
		    0) {
			//printf("Pending interrupts\n");  // may be OK but probably
		}
		sav134 = *(intrupt + 134);
		*(intrupt + 137) = sav134;
		sav132 = *(intrupt + 132);	// save current interrupts
		*(intrupt + 135) = sav132;	// disable active interrupts
		sav133 = *(intrupt + 133);
		*(intrupt + 136) = sav133;
	} else	{		

        // flag = 1 enable
		if (sav132 == 0) {
			printf("Interrupts not disabled\n");
			return (0);
		}

		*(intrupt + 132) = sav132;	// restore saved interrupts
		*(intrupt + 133) = sav133;
		*(intrupt + 134) = sav134;
		sav132 = 0;	// indicates interrupts enabled
	}
	return (1);
}
