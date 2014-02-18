/*
ADAFRUIT RETROGAME UTILITY: remaps buttons on Raspberry Pi GPIO header
to virtual USB keyboard presses.  Great for classic game emulators!
Retrogame is interrupt-driven and efficient (usually under 0.3% CPU use)
and debounces inputs for glitch-free gaming.

Connect one side of button(s) to GND pin (there are several on the GPIO
header, but see later notes) and the other side to GPIO pin of interest.
Internal pullups are used; no resistors required.  Avoid pins 8 and 10;
these are configured as a serial port by default on most systems (this
can be disabled but takes some doing).  GPIO 18 should likewise be
avoided; it's shared with audio output.  Pin configuration is currently
set in global table; no config file yet.  See later comments.

Must be run as root, i.e. 'sudo retrogame' or configure init scripts to
launch automatically at system startup.

Requires uinput kernel module.  This is typically present on popular
Raspberry Pi Linux distributions but not enabled by default.  To enable,
either type:

    sudo modprobe uinput

Or, to make this persistent between reboots, add a line to /etc/modules:

    uinput

Written by Phil Burgess for Adafruit Industries, distributed under BSD
License.  Adafruit invests time and resources providing this open source
code, please support Adafruit and open-source hardware by purchasing
products from Adafruit!


Copyright (c) 2013 Adafruit Industries.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/mman.h>
#include <linux/input.h>
#include <linux/uinput.h>





struct {
	int pin;
	int key;
} io[] = {
//	  Input    Output (from /usr/include/linux/input.h)
	
	{ 21,      KEY_VOLUMEUP       },
	{ 17,      KEY_VOLUMEDOWN     }
};


// START HERE ------------------------------------------------------------
// This table remaps GPIO inputs to keyboard values.  In this initial
// implementation there's a 1:1 relationship (can't attach multiple keys
// to a button) and the list is fixed in code; there is no configuration
// file.  Buttons physically connect between GPIO pins and ground.  There
// are only a few GND pins on the GPIO header, so a breakout board is
// often needed.  If you require just a couple extra ground connections
// and have unused GPIO pins, set the corresponding key value to GND to
// create a spare ground point.

#define max_encoders 8
#define GND -1
#define BCM2708_PERI_BASE 0x20000000
#define GPIO_BASE         (BCM2708_PERI_BASE + 0x200000)
#define BLOCK_SIZE        (4*1024)
#define GPPUD             (0x94 / 4)
#define GPPUDCLK0         (0x98 / 4)




#define IOLEN (sizeof(io) / sizeof(io[0])) // io[] table size



struct encoder
{
    int pin_a;
    int pin_b;
    int pin_a_value;
    int pin_b_value;
    volatile long value;
    volatile int lastEncoded;
};

//Pre-allocate encoder objects on the stack so we don't have to 
//worry about freeing them
struct encoder encoders[max_encoders];

/*
  Should be run for every rotary encoder you want to control
  Returns a pointer to the new rotary encoder structer
  The pointer will be NULL is the function failed for any reason
*/





// A few globals ---------------------------------------------------------

char
  *progName,                         // Program name (for error reporting)
   sysfs_root[] = "/sys/class/gpio", // Location of Sysfs GPIO files
   running      = 1;                 // Signal handler will set to 0 (exit)
volatile unsigned int
  *gpio;                             // GPIO register table


// A few arrays here are declared with IOLEN elements, even though
	// values aren't needed for io[] members where the 'key' value is
	// GND.  This simplifies the code a bit -- no need for mallocs and
	// tests to create these arrays -- but may waste a handful of
	// bytes for any declared GNDs.
	char                   buf[50],         // For sundry filenames
	                       c;               // Pin input value ('0'/'1')
	int                    fd,              // For mmap, sysfs, uinput
	                       i, j,            // Asst. counter
	                       bitmask,         // Pullup enable bitmask
	                       timeout = -1,    // poll() timeout
	                       intstate[IOLEN], // Last-read state
	                       extstate[IOLEN]; // Debounced state
	volatile unsigned char shortWait;       // Delay counter
	struct uinput_user_dev uidev;           // uinput device
	
    struct input_event     keyEv, synEv;    // uinput events
    struct uinput_user_dev uidev;           // uinput device
	struct pollfd          p[IOLEN];        // GPIO file descriptors


    
// FUNCTIONS
void setup_uinput();
void setupGPIO();
struct encoder *setupencoder(int pin_a, int pin_b); 
    
// Some utility functions ------------------------------------------------

struct encoder *setupencoder(int pin_a, int pin_b)
{
    
    struct encoder *newencoder = encoders;
    newencoder->pin_a = pin_a;
    newencoder->pin_b = pin_b;
    newencoder->pin_a_value = 0;
    newencoder->pin_b_value = 0;    
    newencoder->value = 0;
    newencoder->lastEncoded = 0;
    return newencoder;
}


int updateEncoder(struct encoder *encoder)
{
   
    
        int MSB = encoder->pin_a_value;
        int LSB = encoder->pin_b_value;

        int encoded = (MSB << 1) | LSB;
        int sum = (encoder->lastEncoded << 2) | encoded;

        encoder->lastEncoded = encoded;
        
        if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) return -1;
        if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) return 1;

        return 0;
    
}

// Set one GPIO pin attribute through the Sysfs interface.
int pinConfig(int pin, char *attr, char *value) {
	char filename[50];
	int  fd, w, len = strlen(value);
	sprintf(filename, "%s/gpio%d/%s", sysfs_root, pin, attr);
	if((fd = open(filename, O_WRONLY)) < 0) return -1;
	w = write(fd, value, len);
	close(fd);
	return (w != len); // 0 = success
}

// Un-export any Sysfs pins used; don't leave filesystem cruft.  Also
// restores any GND pins to inputs.  Write errors are ignored as pins
// may be in a partially-initialized state.
void cleanup() {
	char buf[50];
	int  fd, i;
	sprintf(buf, "%s/unexport", sysfs_root);
	if((fd = open(buf, O_WRONLY)) >= 0) {
		for(i=0; i<IOLEN; i++) {
			// Restore GND items to inputs
			if(io[i].key == GND)
				pinConfig(io[i].pin, "direction", "in");
			// And un-export all items regardless
			sprintf(buf, "%d", io[i].pin);
			write(fd, buf, strlen(buf));
		}
		close(fd);
	}
}

// Quick-n-dirty error reporter; print message, clean up and exit.
void err(char *msg) {
	printf("%s: %s.  Try 'sudo %s'.\n", progName, msg, progName);
	cleanup();
	exit(-1);
}

// Interrupt handler -- set global flag to abort main loop.
void signalHandler(int n) {
	running = 0;
}


// Main stuff ------------------------------------------------------------



int main(int argc, char *argv[]) {


    struct encoder *encoder = setupencoder(21,17);

	
	progName = argv[0];             // For error reporting
	signal(SIGINT , signalHandler); // Trap basic signals (exit cleanly)
	signal(SIGKILL, signalHandler);


	
    setupGPIO();

	// ----------------------------------------------------------------
	// Set up uinput

	setup_uinput();

	// 'fd' is now open file descriptor for issuing uinput events


	// ----------------------------------------------------------------
	// Monitor GPIO file descriptors for button events.  The poll()
	// function watches for GPIO IRQs in this case; it is NOT
	// continually polling the pins!  Processor load is near zero.
    int a=0,b=0;
    int state=0;
    int count=0;
    
	while(running) { // Signal handler can set this to 0 to exit
		// Wait for IRQ on pin (or timeout for button debounce)
		
        
        if(poll(p, 2, timeout) > 0) { // If IRQ...           
            lseek(p[0].fd, 0, SEEK_SET);
            read(p[0].fd, &a, 1);
            lseek(p[1].fd, 0, SEEK_SET);
            read(p[1].fd, &b, 1);
            encoder->pin_a_value=49-a;
            encoder->pin_b_value=49-b;
            state=1;
            //we get 3 values for each position
            if (count==2)
            {
                //the sum of the inputs will be either negative or positive.
                intstate[0]+=updateEncoder(encoder);
                count=0;
            }
            count++;

            

			timeout = 10; // Set timeout for debounce
		} else  { 
            c=0;
            if(state != 0 ) {
                if (intstate[0]<0)
                {
                    keyEv.code=io[0].key;
                    //printf("key down\n");
                    c=1;
                }
                if (intstate[0]>0)
                {
                    keyEv.code=io[1].key; 
                    //printf("key up\n");
                    c=1; 
                }
                if (c)
                {
                    keyEv.value = 1;
                    write(fd, &keyEv, sizeof(keyEv));                       
                    keyEv.value = 0;
                    write(fd, &keyEv, sizeof(keyEv));
                    write(fd, &synEv, sizeof(synEv));
                }
                
                extstate[0] = intstate[0];
                intstate[0]=0;                 
                state=0;
                
             
            }

           
            c=0;
            timeout = -1; // Return to normal IRQ monitoring
		}
	}

	// ----------------------------------------------------------------
	// Clean up

	ioctl(fd, UI_DEV_DESTROY); // Destroy and
	close(fd);                 // close uinput
	cleanup();                 // Un-export pins

	puts("Done.");

	return 0;
}

void setupGPIO()
{
// ----------------------------------------------------------------
	// Although Sysfs provides solid GPIO interrupt handling, there's
	// no interface to the internal pull-up resistors (this is by
	// design, being a hardware-dependent feature).  It's necessary to
	// grapple with the GPIO configuration registers directly to enable
	// the pull-ups.  Based on GPIO example code by Dom and Gert van
	// Loo on elinux.org

	if((fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0)
		err("Can't open /dev/mem");
	gpio = mmap(            // Memory-mapped I/O
	  NULL,                 // Any adddress will do
	  BLOCK_SIZE,           // Mapped block length
	  PROT_READ|PROT_WRITE, // Enable read+write
	  MAP_SHARED,           // Shared with other processes
	  fd,                   // File to map
	  GPIO_BASE );          // Offset to GPIO registers
	close(fd);              // Not needed after mmap()
	if(gpio == MAP_FAILED) err("Can't mmap()");
	// Make combined bitmap of pullup-enabled pins:
	for(bitmask=i=0; i<IOLEN; i++)
		if(io[i].key != GND) bitmask |= (1 << io[i].pin);
	gpio[GPPUD]     = 2;                    // Enable pullup
	for(shortWait=150;--shortWait;);        // Min 150 cycle wait
	gpio[GPPUDCLK0] = bitmask;              // Set pullup mask
	for(shortWait=150;--shortWait;);        // Wait again
	gpio[GPPUD]     = 0;                    // Reset pullup registers
	gpio[GPPUDCLK0] = 0;
	(void)munmap((void *)gpio, BLOCK_SIZE); // Done with GPIO mmap()


	// ----------------------------------------------------------------
	// All other GPIO config is handled through the sysfs interface.

	sprintf(buf, "%s/export", sysfs_root);
	if((fd = open(buf, O_WRONLY)) < 0) // Open Sysfs export file
		err("Can't open GPIO export file");
	for(i=j=0; i<IOLEN; i++) { // For each pin of interest...
		sprintf(buf, "%d", io[i].pin);
		write(fd, buf, strlen(buf));             // Export pin
		pinConfig(io[i].pin, "active_low", "0"); // Don't invert
		if(io[i].key == GND) {
			// Set pin to output, value 0 (ground)
			if(pinConfig(io[i].pin, "direction", "out") ||
			   pinConfig(io[i].pin, "value"    , "0"))
				err("Pin config failed (GND)");
		} else {
			// Set pin to input, detect rise+fall events
			if(pinConfig(io[i].pin, "direction", "in") ||
			   pinConfig(io[i].pin, "edge"     , "both"))
				err("Pin config failed");
			// Get initial pin value
			sprintf(buf, "%s/gpio%d/value",
			  sysfs_root, io[i].pin);
			// The p[] file descriptor array isn't necessarily
			// aligned with the io[] array.  GND keys in the
			// latter are skipped, but p[] requires contiguous
			// entries for poll().  So the pins to monitor are
			// at the head of p[], and there may be unused
			// elements at the end for each GND.  Same applies
			// to the intstate[] and extstate[] arrays.
			if((p[j].fd = open(buf, O_RDONLY)) < 0)
				err("Can't access pin value");
			intstate[j] = 0;
			if((read(p[j].fd, &c, 1) == 1) && (c == '0'))
				intstate[j] = 1;
			extstate[j] = intstate[j];
			p[j].events  = POLLPRI; // Set up poll() events
			p[j].revents = 0;
			j++;
		}
	} // 'j' is now count of non-GND items in io[] table
	close(fd); // Done exporting
}
void setup_uinput() 
{
    int i=0;
    if((fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK)) < 0)
		err("Can't open /dev/uinput");
	if(ioctl(fd, UI_SET_EVBIT, EV_KEY) < 0)
		err("Can't SET_EVBIT");
	for(i=0; i<IOLEN; i++) {
		if(io[i].key != GND) {
			if(ioctl(fd, UI_SET_KEYBIT, io[i].key) < 0)
				err("Can't SET_KEYBIT");
		}
	}
	memset(&uidev, 0, sizeof(uidev));
	snprintf(uidev.name, UINPUT_MAX_NAME_SIZE, "retrogame");
	uidev.id.bustype = BUS_USB;
	uidev.id.vendor  = 0x1;
	uidev.id.product = 0x1;
	uidev.id.version = 1;
	if(write(fd, &uidev, sizeof(uidev)) < 0)
		err("write failed");
	if(ioctl(fd, UI_DEV_CREATE) < 0)
		err("DEV_CREATE failed");
	// Initialize input event structures
	memset(&keyEv, 0, sizeof(keyEv));
	keyEv.type  = EV_KEY;
	memset(&synEv, 0, sizeof(synEv));
	synEv.type  = EV_SYN;
	synEv.code  = SYN_REPORT;
	synEv.value = 0;
}
