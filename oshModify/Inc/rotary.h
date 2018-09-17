/*
 * ----------------------------------------------------------------------------------
 * @file  		: rotary.h
 * @author 		: Dmitry Skulkin <dmitry.skulkin@gmail.com>
 * @version		: 0.1
 * @brief 		: Rotary Encoders support library
 *
 *
 * ----------------------------------------------------------------------------------
 *	 Copyright (c) 2016 Dmitry Skulkin <dmitry.skulkin@gmail.com>					*
 *																					*
 *	Permission is hereby granted, free of charge, to any person obtaining a copy	*
 *	of this software and associated documentation files (the "Software"), to deal	*
 *	in the Software without restriction, including without limitation the rights	*
 *	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell		*
 *	copies of the Software, and to permit persons to whom the Software is			*
 *	furnished to do so, subject to the following conditions:						*
 *																					*
 *	The above copyright notice and this permission notice shall be included in all	*
 *	copies or substantial portions of the Software.									*
 *																					*
 *	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR		*
 *	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,		*
 *	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE		*
 *	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER			*
 *	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,	*
 *	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE	*
 *	SOFTWARE.																		*
 * ----------------------------------------------------------------------------------
 * */


#ifndef ROTARY_H_
#define ROTARY_H_

#include <stdint.h>
#include "periph_init.h"

//#define ROTTIME 50
//#define ROTDEBOUNCE 10

#define DIR_CCW 0x10
#define DIR_CW 0x20

unsigned char rotary_process(uint8_t _i, pintype type, uint32_t *PINA_Addr, uint32_t *PINB_Addr, uint16_t PINA, uint16_t PINB);
void CheckRotaries(void);

struct rots {
	volatile uint8_t pressed;
	volatile uint64_t time_pressed;
	volatile uint8_t state;
};


#endif /* ROTARY_H_ */
