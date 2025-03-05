/*

Copyright 2024 Simon Peter

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS”
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */


#ifndef TIMER_BASE_H
#define TIMER_BASE_H

//System-Includes
#include "stm32h7xx.h"


struct TimBase {
	TimBase(TIM_TypeDef* instance, uint32_t _clkFrequ, GPIO_TypeDef** _gpiox, uint32_t* _px, uint32_t* _alternate)
		: timInst(instance), clkFrequ(_clkFrequ), gpiox(_gpiox), px(_px), alternate(_alternate)
	{}

    virtual ~TimBase(){}

	TIM_TypeDef* timInst;		// Configuration structure for the (general) time
	uint32_t clkFrequ;			// Timer counting frequency

	GPIO_TypeDef** gpiox;		// Port of the timer output pins
	uint32_t* px;				// Output pins
	uint32_t* alternate;
};


//-------------- Base timer functions ------------------------------------------------------------------------------------------------------

// Initialise the gpio output pins for the timer
void timInitPins(const TimBase& tim);

// De-Initialise the gpio pins for the timer to input with pull-down
void timDeInitPins(const TimBase& tim);

// CEN[0] = 1: Activate timer
inline void timActivate(TimBase& tim) { tim.timInst->CR1 |= TIM_CR1_CEN; }

// CEN[0] = 0: Deactivate timer
inline void timDeactivate(TimBase& tim) { tim.timInst->CR1 &= ~TIM_CR1_CEN; }


#endif
