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

#ifndef MIDICLASS_h
#define MIDICLASS_h

#include <Arduino.h>

#ifdef ARDUINO_BABYGNUSBUINO2
#include <GnusbuinoMIDIConfig.h>        // custom config for Babygnusbuino2 user
#endif

#include <inttypes.h>
#include <stdio.h> // for size_t
#include <avr/wdt.h>
#include <util/delay.h>
#include <usbdrv.h>

#include <GnusbuinoMIDIdescriptor.h>
#include <GnusbuinoMIDInotes.h>


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
#define MCUCSR   MCUSR
#endif

// ==============================================================================
// MIDI STUFF
// ------------------------------------------------------------------------------

typedef struct _midi_msg
{
    uchar cn : 4;
    uchar cin : 4;
    uchar byte[3];
} midi_msg;


/******************************************************************************
 * Definitions
 ******************************************************************************/
// MIDI Status Bytes

#define MIDI_NOTEOFF			0x80
#define MIDI_NOTEON				0x90
#define MIDI_POLYAFTERTOUCH		0xA0
#define MIDI_CONTROLCHANGE		0xB0
#define MIDI_PROGRAMCHANGE		0xC0
#define MIDI_CHANNELAFTERTOUCH	0xD0
#define MIDI_PITCHBEND			0xE0

//Realtime Messages
#define MIDI_QUARTERFRAME		0xF1
#define MIDI_SONGPOS			0xF2
#define MIDI_SONGSELECT			0xF3
#define MIDI_TIMINGCLOCK		0xF8
#define MIDI_START				0xFA
#define MIDI_CONTINUE			0xFB
#define MIDI_STOP				0xFC

#if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny84__) //fix buffer for attiny84 by stahl
    #define MIDI_MAX_BUFFER		10
#else
    #define MIDI_MAX_BUFFER		64
#endif

typedef struct
{
    unsigned char command;
    unsigned char key;
    unsigned char value;
} MIDIMessage;

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2


/******************************************************************************
 * MIDI Class
 ******************************************************************************/

class MIDIClass
{

public:
    void write(uint8_t,uint8_t,uint8_t);
    uint8_t read(MIDIMessage*);
    size_t print(const char *);
    size_t print(char, int = DEC);
    size_t print(unsigned char, int = DEC);
    size_t print(int, int = DEC);
    size_t print(unsigned int, int = DEC);
    size_t print(long, int = DEC);
    size_t print(unsigned long, int = DEC);
    size_t print(double, int = 2);

    size_t println(void);
    size_t println(char, int = DEC);
    size_t println(const char *);
    size_t println(unsigned char, int = DEC);
    size_t println(int, int = DEC);
    size_t println(unsigned int, int = DEC);
    size_t println(long, int = DEC);
    size_t println(unsigned long, int = DEC);
    size_t println(double, int = 2);

    void init();
    void sendMIDI(void);
    void flush(void);
    void delay(unsigned long ms);
    void restartToBootloader(void);

    void receiveMIDI(uint8_t,uint8_t,uint8_t);

private:
    unsigned char _midiOutData[4];

    unsigned char _midiSendEnqueueIdx;
    unsigned char _midiSendDequeueIdx;
    unsigned char _midiSendQueue [MIDI_MAX_BUFFER * 3];

    unsigned char _midiRecvEnqueueIdx;
    unsigned char _midiRecvDequeueIdx;
    unsigned char _midiRecvQueue [MIDI_MAX_BUFFER * 3];

    char * _sysex_buffer;
    unsigned char _sysex_idx;
    unsigned char _sysex_len;

};


extern void handleNoteOn(unsigned char note);
extern void handleNoteOff(unsigned char note);
void sendControlChange(unsigned char controller,unsigned char value);
void sendNote(unsigned char key, unsigned char velocity);

extern MIDIClass MIDI;

#endif
