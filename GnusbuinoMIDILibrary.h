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

#ifndef GNUSBUINOMIDILIB_h
#define GNUSBUINOMIDILIB_h

#include <Arduino.h>

#include <usbdrv.h>
#include <GnusbuinoMIDI.h>
#include <GnusbuinoMIDIdescriptor.h>

#include <avr/wdt.h>
#include <util/delay.h>


// ==============================================================================
// Constants
// ------------------------------------------------------------------------------

// Usb request to start Bootloader for Software updates
#define STATUS_LED_PORT PORTD
#define GNUSBCORE_CMD_SET_SMOOTHING			0xf0
#define GNUSBCORE_CMD_START_BOOTLOADER 		0xf8

// this is from original midi_gnusb.h
#define ADC_MUX_MASK			0x0F
#define ADC_PRESCALE_DIV64		0x06	///< 0x06 -> CPU clk/64
#define ADC_PRESCALE_MASK		0x07

#ifndef MCUCSR          /* compatibility between ATMega16 and ATMega644 */
#   define MCUCSR   MCUSR
#endif

#define delay(x) _delay_usb(x)

// ==============================================================================
// MIDI STUFF
// ------------------------------------------------------------------------------

typedef struct _midi_msg {
	uchar cn : 4;
	uchar cin : 4;
	uchar byte[3];
} midi_msg;


// ------------------------------------------------------------------------------
// - Defined in main
// ------------------------------------------------------------------------------

extern void handleNoteOn(unsigned char note);
extern void handleNoteOff(unsigned char note);
void sendControlChange(unsigned char controller,unsigned char value);
void sendNote(unsigned char key, unsigned char velocity);

void usbMidiInit();
void startBootloader(void);
unsigned char usbFunctionDescriptor(usbRequest_t * rq);
unsigned char usbFunctionSetup(unsigned char data[8]);
unsigned char usbFunctionRead(unsigned char * data, unsigned char len);
unsigned char usbFunctionWrite(unsigned char * data, unsigned char len);
void usbFunctionWriteOut(unsigned char * data, unsigned char len);
void doPeriodical(void);
void adInit(void);
void _delay_usb(unsigned long ms);


#endif

