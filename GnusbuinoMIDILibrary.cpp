/*
  GnusbuinoMIDI.h - MIDI over USB library for the Gnusbuino
  http://gnusb.sourceforge.net
  
  Copyright (c) 2012 Michael Egger.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "GnusbuinoMIDILibrary.h"

MIDIClass MIDI;

//uchar midiMsg[8];

// ------------------------------------------------------------------------------
// - Start Bootloader
// ------------------------------------------------------------------------------
// dummy function doing the jump to bootloader section (Adress 1C00 on Atmega644)

void usbMidiRestartToBootloader(void) {

		cli();							// turn off interrupts
		wdt_disable();					// disable watchdog timer
		usbDeviceDisconnect(); 			// disconnect gnusb from USB bus
		_delay_usb(100);
		
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

			usbMidiRestartToBootloader();
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
		//Sysex message  07 F0 08 F7
		if (	
			data[0] == 0x07 && data[1] == 0xF0 && data[2] == 0x08 && data[3] == 0xF7
			)
		{
			usbMidiRestartToBootloader();
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
// - usbMidiSend
// ------------------------------------------------------------------------------
// stuff that has do be done often in the loop and not forgotten while delaying


void usbMidiSend(void) {

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
void usbMidiADCinit(void){



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



void usbMidiInit()
{
	MCUCSR = (1 << PORF);			// set power on reset flag just to be sure
	usbMidiADCinit();

	wdt_enable(WDTO_1S);	// enable watchdog timer
	
	#if defined(__AVR_ATtiny85__)
		//DDRB = (1 << 2); 	//PB2 is output
	#else
		// set PORT D Directions -> 1110 0000, output 0 on unconnected PD7
		DDRD = 0xe0; 	// 1110 0000 -> set PD0..PD4 to inputs -> USB pins
		PORTD = 0x70; 	// set Pullup for Bootloader Jumper, no pullups on USB pins -> 0111 0000
	#endif

	unsigned char   i = 0;

    usbInit();
  
    // enforce USB re-enumerate: 

	cli();    
    usbDeviceDisconnect();  // do this while interrupts are disabled 
    while(--i){         // fake USB disconnect for > 250 ms 
        wdt_reset();
        _delay_usb(1);
    }

    usbDeviceConnect();
    sei();
		
	//setup();
}



// ------------------------------------------------------------------------------
// - delay usb
// ------------------------------------------------------------------------------
// taken from wiring.cpp (midibabygnusbuino)

void _delay_usb(unsigned long ms)
{
	uint16_t start = (uint16_t)micros();

	while (ms > 0) {	
		if (((uint16_t)micros() - start) >= 1000) {
			usbMidiSend();
			ms--;
			start += 1000;
		}
	}
}


