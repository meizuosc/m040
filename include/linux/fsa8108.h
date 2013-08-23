/*
 * fsa8108.h -- FSA8108 Jack detection driver
 *
 * Copyright (C) 2012 Fairchild semiconductor Co.Ltd
 * Author: Chris Jeong <Chris.jeong@fairchildsemi.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __FSA8108_JACK_DETECT_H__
#define __FSA8108_JACK_DETECT_H__

typedef unsigned char BYTE;

enum {
	FSA_JACK_NO_DEVICE,				
	FSA_HEADSET_4POLE,				
	FSA_HEADSET_3POLE				
};

extern void fsa8108_MP3_mode(int onoff);
extern void fsa8108_LDO_output(int onoff);
#endif
