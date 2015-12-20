#ifndef MIDIDESC_h
#define MIDIDESC_h



/* Name: desc_midi.inc
 * Project: V-USB MIDI device on Low-Speed USB
 * Author: Martin Homuth-Rosemann
 * Creation Date: 2008-03-11
 * Copyright: (c) 2008,2009,2010 by Martin Homuth-Rosemann.
 * License: GPL.
 * Version: 0.03
 */

// don't include (mandatory) dummy AudioControl descriptor
#define WITHOUT_AUDIO_CONTROL

// This descriptor is based on http://www.usb.org/developers/devclass_docs/midi10.pdf
// constants from midi10.pdf

// Appendix A

// A.1 MS Class-Specific Interface Descriptor Subtypes
#define M10_MS_DESCRIPTOR_UNDEFINED	0x00
#define M10_MS_HEADER			0x01
#define M10_MIDI_IN_JACK		0x02
#define M10_MIDI_OUT_JACK		0x03
#define M10_ELEMENT			0x04

// A.2 MS Class-Specific Endpoint Descriptor Subtypes
#define M10_DESCRIPTOR_UNDEFINED	0x00
#define M10_MS_GENERAL			0x01

// A.3 MS MIDI IN and OUT Jack types
#define M10_JACK_TYPE_UNDEFINED		0x00
#define M10_EMBEDDED			0x01
#define M10_EXTERNAL			0x02


#define M10_DEVICE			0x01
#define M10_CONFIGURATION		0x02
#define M10_INTERFACE			0x04
#define M10_ENDPOINT			0x05
#define M10_CS_INTERFACE		0x24
#define M10_CS_ENDPOINT			0x25

#define M10_BULK			0x02
#define M10_INTERRUPT			0x03

#define M10_AUDIO			0x01
#define M10_AUDIO_CONTROL		0x01
#define M10_MIDISTREAMING		0x03

#define M10_UNUSED			0x00

#define M10_DIR_IN			0x80
#define M10_DIR_OUT			0x00


// This descriptor is based on http://www.usb.org/developers/devclass_docs/midi10.pdf
//
// Appendix B. Example: Simple MIDI Adapter (Informative)
// B.1 Device Descriptor
//
static PROGMEM const char deviceDescrMIDI[] = {	/* USB device descriptor */
    18,			/* sizeof(usbDescriptorDevice): length of descriptor in bytes */
    M10_DEVICE,		/* descriptor type */
    0x10, 0x01,		/* bcdUSB USB version supported */
    M10_UNUSED,		/* device class: defined at interface level */
    M10_UNUSED,		/* subclass */
    M10_UNUSED,		/* protocol */
    8,			/* bMaxPacketSize max packet size */
    0xc0, 0x16,		/* idVendor  16c0 VOTI 2 bytes */
    0xe4, 0x05,		/* idProduct 05e4 MIDI 2 bytes */
    0x03, 0x00,		/* bcdDevice 0.03 2 bytes */
    1,			/* iManufacturer manufacturer string index */
    2,			/* iProduct product string index */
    M10_UNUSED,		/* serial number string index */
    1,			/* number of configurations */
};



// B.2 Configuration Descriptor
static PROGMEM const char configDescrMIDI[] = {	/* USB configuration descriptor */
    9,			/* sizeof(usbDescrConfig): length of descriptor in bytes */
    USBDESCR_CONFIG,	/* descriptor type */

#ifdef WITHOUT_AUDIO_CONTROL
    18+65, 0,		/* 9+9+65 total length of data returned */
                /* (including inlined descriptors) */
                /* B.2 + (B.3) + B.4.1 + length in B.4.2 */
    1,			/* bNumInterfaces AC + MS = 2 */
                /* (number of interfaces in this configuration) */
#else
    36+65, 0,		/* 9+(9)+(9)+9+65 total length of data returned */
                /* (including inlined descriptors) */
                /* B.2 + (B.3) + B.4.1 + length in B.4.2 */
    2,			/* bNumInterfaces AC + MS = 2 */
                /* (number of interfaces in this configuration) */
#endif

    1,			/* index of this configuration */
    M10_UNUSED,		/* configuration name string index */
#if USB_CFG_IS_SELF_POWERED
    (1 << 7) | USBATTR_SELFPOWER,	/* attributes */
#else
    (1 << 7),		/* attributes */
#endif
    USB_CFG_MAX_BUS_POWER / 2,	/* max USB current in 2mA units */


#ifndef WITHOUT_AUDIO_CONTROL
//////////////////////////////////////////
// TEST: leave out AudioControl		//
//	changes:			//
//	set total length in B.2		//
// 	set bNumInterfaces=1 in B.2	//
//	set bInterfaceNumber=0 in B.4.1	//
//////////////////////////////////////////
//
// B.3 AudioControl Interface Descriptors
// The AudioControl interface describes the device structure (audio function topology)
// and is used to manipulate the Audio Controls. This device has no audio function
// incorporated. However, the AudioControl interface is mandatory and therefore both
// the standard AC interface descriptor and the classspecific AC interface descriptor
// must be present. The class-specific AC interface descriptor only contains the header
// descriptor.

// B.3.1 Standard AC Interface Descriptor
// The AudioControl interface has no dedicated endpoints associated with it. It uses the
// default pipe (endpoint 0) for all communication purposes. Class-specific AudioControl
// Requests are sent using the default pipe. There is no Status Interrupt endpoint provided.
    /* AC interface descriptor follows inline: */
    9,			/* sizeof(usbDescrInterface): length of descriptor in bytes */
    USBDESCR_INTERFACE,	/* descriptor type */
    0,			/* index of this interface */
    0,			/* alternate setting for this interface */
    0,			/* endpoints excl 0: number of endpoint descriptors to follow */
    M10_AUDIO,		/* bInterfaceClass*/
    M10_AUDIO_CONTROL,	/* bInterfaceSubclass*/
    M10_UNUSED,		/* bInterfaceProtocol*/
    M10_UNUSED,		/* string index for interface */

// B.3.2 Class-specific AC Interface Descriptor
// The Class-specific AC interface descriptor is always headed by a Header descriptor
// that contains general information about the AudioControl interface. It contains all
// the pointers needed to describe the Audio Interface Collection, associated with the
// described audio function. Only the Header descriptor is present in this device
// because it does not contain any audio functionality as such.
    /* AC Class-Specific descriptor */
    9,			/* sizeof(usbDescrCDC_HeaderFn): length of descriptor in bytes */
    M10_CS_INTERFACE,	/* descriptor type */
    M10_MS_HEADER,		/* header functional descriptor */
    0x0, 0x01,		/* bcdADC */
    9, 0,			/* wTotalLength */
    1,			/* bInCollection */
    1,			/* baInterfaceNr(1) */

#endif

// B.4 MIDIStreaming Interface Descriptors

// B.4.1 Standard MS Interface Descriptor
    /* interface descriptor follows inline: */
    9,			/* length of descriptor in bytes */
    USBDESCR_INTERFACE,	/* descriptor type */
#ifdef WITHOUT_AUDIO_CONTROL
    0,			/* bInterfaceNumber index of this interface */
#else
    1,			/* bInterfaceNumber index of this interface */
#endif
    0,			/* alternate setting for this interface */
    2,			/* endpoints excl 0: number of endpoint descriptors to follow */
    M10_AUDIO,		/* bInterfaceClass */
    M10_MIDISTREAMING,	/* bInterfaceSubclass */
    M10_UNUSED,		/* bInterfaceProtocol*/
    M10_UNUSED,		/* iInterface string index for interface */

// B.4.2 Class-specific MS Interface Descriptor
    /* MS Class-Specific descriptor */
    7,			/* bLength */
    M10_CS_INTERFACE,	/* bDescritorType */
    M10_MS_HEADER,		/* header functional descriptor */
    0x0, 0x01,		/* bcdADC */
    65, 0,			/* wTotalLength B.4.2 .. B.6.2 */
                /* = 7 + 6 + 6 + 9 + 9 + 9 + 5 + 9 + 5 */

// B.4.3 MIDI IN Jack Descriptor
    6,			/* bLength */
    M10_CS_INTERFACE,	/* descriptor type */
    M10_MIDI_IN_JACK,	/* desc subtype */
    M10_EMBEDDED,		/* bJackType */
    1,			/* bJackID */
    M10_UNUSED,		/* iJack */

    6,			/* bLength */
    M10_CS_INTERFACE,	/* descriptor type */
    M10_MIDI_IN_JACK,	/* desc subtype */
    M10_EXTERNAL,		/* EXTERNAL bJackType */
    2,			/* bJackID */
    M10_UNUSED,		/* iJack */

//B.4.4 MIDI OUT Jack Descriptor
    9,			/* length of descriptor in bytes */
    M10_CS_INTERFACE,	/* descriptor type */
    M10_MIDI_OUT_JACK,	/* descriptor */
    M10_EMBEDDED,		/* bJackType */
    3,			/* bJackID */
    1,			/* No of input pins */
    2,			/* BaSourceID */
    1,			/* BaSourcePin */
    M10_UNUSED,		/* iJack */

    9,			/* bLength of descriptor in bytes */
    M10_CS_INTERFACE,	/* bDescriptorType */
    M10_MIDI_OUT_JACK,	/* bDescriptorSubtype */
    M10_EXTERNAL,		/* bJackType */
    4,			/* bJackID */
    1,			/* bNrInputPins */
    1,			/* baSourceID (0) */
    1,			/* baSourcePin (0) */
    M10_UNUSED,		/* iJack */


// B.5 Bulk OUT Endpoint Descriptors

//B.5.1 Standard Bulk OUT Endpoint Descriptor
    9,			/* bLenght */
    USBDESCR_ENDPOINT,	/* bDescriptorType = endpoint */
    1 | M10_DIR_OUT,	/* bEndpointAddress OUT endpoint number 1 */
    M10_BULK,		/* bmAttributes: 2:Bulk, 3:Interrupt endpoint */
    8, 0,			/* wMaxPacketSize */
    2,			/* bIntervall in ms */
    M10_UNUSED,		/* bRefresh */
    M10_UNUSED,		/* bSyncAddress */

// B.5.2 Class-specific MS Bulk OUT Endpoint Descriptor
    5,			/* bLength of descriptor in bytes */
    M10_CS_ENDPOINT,	/* bDescriptorType */
    M10_MS_GENERAL,		/* bDescriptorSubtype */
    1,			/* bNumEmbMIDIJack */
    1,			/* baAssocJackID (0) */

//B.6 Bulk IN Endpoint Descriptors

//B.6.1 Standard Bulk IN Endpoint Descriptor
    9,			/* bLenght */
    USBDESCR_ENDPOINT,	/* bDescriptorType = endpoint */
    M10_DIR_IN | 1,		/* bEndpointAddress IN endpoint number 1 */
    M10_BULK,		/* bmAttributes: 2: Bulk, 3: Interrupt endpoint */
    8, 0,			/* wMaxPacketSize */
    2,			/* bIntervall in ms */
    M10_UNUSED,		/* bRefresh */
    M10_UNUSED,		/* bSyncAddress */

// B.6.2 Class-specific MS Bulk IN Endpoint Descriptor
    5,			/* bLength of descriptor in bytes */
    M10_CS_ENDPOINT,	/* bDescriptorType */
    M10_MS_GENERAL,		/* bDescriptorSubtype */
    1,			/* bNumEmbMIDIJack */
    3			/* baAssocJackID (0) */
};


#endif
