#ifndef	__midignusb__
#define __midignusb__
// ==============================================================================
// midi_gnusb.h
// globals and utilities for gnusb - OPEN SOURCE USB SENSOR BOX
//
// License:
// The project is built with AVR USB driver by Objective Development, which is
// published under an own licence based on the GNU General Public License (GPL).
// gnusb is also distributed under this enhanced licence. See Documentation.
//
// target-cpu: ATMega16 @ 12MHz
// created 2007-01-28 Michael Egger me@anyma.ch
// version 2011-10-23 Michael Egger me@anyma.ch
//
// ==============================================================================

// USB driver by Objective Development (see http://www.obdev.at/products/avrusb/index.html)
#include <Arduino.h>


#include <avr/wdt.h>
#include <util/delay.h>

#include "usbdrv.h"
#include "usbconfig.h"

#include "GnusbuinoMIDI.h"	// not very clean, I know
#include "GnusbuinoMIDIdescriptor.h"

//MIDIQueue midiSendQueue;
MIDIClass MIDI;

// ==============================================================================
// Constants
// ------------------------------------------------------------------------------

// Usb request to start Bootloader for Software updates
#define STATUS_LED_PORT PORTD
#define GNUSBCORE_CMD_SET_SMOOTHING			0xf0
#define GNUSBCORE_CMD_START_BOOTLOADER 		0xf8



#ifndef MCUCSR          /* compatibility between ATMega16 and ATMega644 */
#   define MCUCSR   MCUSR
#endif


// ------------------------------------------------------------------------------
// - Defined in main
// ------------------------------------------------------------------------------

//extern void handleNoteOn(unsigned char note);
//extern void handleNoteOff(unsigned char note);



// ------------------------------------------------------------------------------
// - Start Bootloader
// ------------------------------------------------------------------------------
// dummy function doing the jump to bootloader section (Adress 1C00 on Atmega644)

void startBootloader(void) {

		cli();							// turn off interrupts
		wdt_disable();					// disable watchdog timer
		usbDeviceDisconnect(); 			// disconnect gnusb from USB bus
		_delay_ms(100);
		
		USB_INTR_ENABLE = 0;
	    USB_INTR_CFG = 0;       /* also reset config bits */

		wdt_enable(WDTO_30MS);	// enable watchdog timer
			while(1) {			// let WDT reset gnusb
		}
		
}






// ------------------------------------------------------------------------------
// - usbFunctionDescriptor
// ------------------------------------------------------------------------------

unsigned char usbFunctionDescriptor(usbRequest_t * rq)
{

	if (rq->wValue.bytes[1] == USBDESCR_DEVICE) {
		usbMsgPtr = (unsigned char *) deviceDescrMIDI;
		return sizeof(deviceDescrMIDI);
	} else {		/* must be config descriptor */
		usbMsgPtr = (unsigned char *) configDescrMIDI;
		return sizeof(configDescrMIDI);
	}
}

// ------------------------------------------------------------------------------
// - usbFunctionSetup
// ------------------------------------------------------------------------------
// this function gets called when the usb driver receives a non standard request
// that is: our own requests defined in ../common/gnusb_cmds.h
unsigned char usbFunctionSetup(unsigned char data[8])
{
	void *pVoid = data;
	usbRequest_t    *rq = static_cast<usbRequest_t*>(pVoid);

	if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) {	/* class request type */

		/*  Prepare bulk-in endpoint to respond to early termination   */
		if ((rq->bmRequestType & USBRQ_DIR_MASK) ==
		    USBRQ_DIR_HOST_TO_DEVICE) {}
	}
	switch (data[1]) {
// 								----------------------------   Start Bootloader for reprogramming the gnusb    		
		case GNUSBCORE_CMD_START_BOOTLOADER:

			startBootloader();
			break;
				
		default:
			break;
	}
	return 0xff;
}


// ---------------------------------------------------------------------------
//  usbFunctionRead                                                          
// ---------------------------------------------------------------------------

unsigned char usbFunctionRead(unsigned char * data, unsigned char len)
{

//	statusLedToggle(StatusLed_Yellow);

//???? thats from http://cryptomys.de/horo/V-USB-MIDI/index.html
	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;
	data[4] = 0;
	data[5] = 0;
	data[6] = 0;

	return 7;
}


/*---------------------------------------------------------------------------*/
/* usbFunctionWrite                                                          */
/*---------------------------------------------------------------------------*/

unsigned char usbFunctionWrite(unsigned char * data, unsigned char len)
{
	return 1;
}

/*---------------------------------------------------------------------------*/
/* usbFunctionWriteOut                                                       */
/*                                                                           */
/* this Function is called if a MIDI Out message (from PC) arrives.          */
/*                                                                           */
/*---------------------------------------------------------------------------*/
void usbFunctionWriteOut(unsigned char * data, unsigned char len)
{
	if (len == 4)
	{
		//00000000  07 F0 08 F7              . ð . ÷          
		if (	
			data[0] == 0x07 && data[1] == 0xF0 && data[2] == 0x08 && data[3] == 0xF7
			)
		{
			startBootloader();
		}
	}


	return;

	// statusLedBlink(StatusLed_Yellow);

	while (len >= sizeof(midi_msg)) {
	
		midi_msg* msg = (midi_msg*)data;
		
		MIDI.receiveMIDI(msg->byte[0],msg->byte[1],msg->byte[2]);

		data += sizeof(midi_msg);
		len -= sizeof(midi_msg);
	}
}



// ------------------------------------------------------------------------------
// - doPeriodical
// ------------------------------------------------------------------------------
// stuff that has do be done often in the loop and not forgotten while delaying


void doPeriodical(void) {

		// if (blinkstop) {
		// 	if (millis() >= blinkstop) {
		// 		statusLedOff(StatusLed_Yellow);
		// 		blinkstop = 0;
		// 	}
		// }	
		
		usbPoll();
		MIDI.sendMIDI();        


		wdt_reset();
}		




// ------------------------------------------------------------------------------
// --------------------- Init AD Converter
// ------------------------------------------------------------------------------
void adInit(void){



	ADCSRA |= (1 << ADEN);				// enable ADC (turn on ADC power)
	ADCSRA &= ~(1 << ADATE);			// default to single sample convert mode
										// Set ADC-Prescaler (-> precision vs. speed)

	ADCSRA = ((ADCSRA & ~ADC_PRESCALE_MASK) | ADC_PRESCALE_DIV64); // Set ADC Reference Voltage to AVCC
	
	#if defined(__AVR_ATtiny85__)
		ADMUX = 0;	// make sure we don't have AREF on PB0 which is used as a usb pullup
	#else			// for bigger chips, use AREF with capacitor
		ADMUX |= (1 << REFS0);		
		ADMUX &= ~(1 << REFS1);
	#endif

	ADCSRA &= ~(1 << ADLAR);				// set to right-adjusted result//	sbi(ADCSRA, ADIE);				// enable ADC interrupts
	ADCSRA &= ~(1 << ADIE);				// disable ADC interrupts
//	ad_initialized = 1;
}



// ==============================================================================
// MIDI STUFF
// ------------------------------------------------------------------------------

// uchar midiMsg[8];


// typedef struct _midi_msg {
// 	uchar cn : 4;
// 	uchar cin : 4;
// 	uchar byte[3];
// } midi_msg;

// void sendControlChange(unsigned char controller,unsigned char value);
// void sendNote(unsigned char key, unsigned char velocity);


#endif
