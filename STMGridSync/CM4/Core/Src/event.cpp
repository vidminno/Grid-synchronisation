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


//System-Includes
#include "stm32h7xx.h"
#include "main.h"

// User-Includes
#include "event.h"


void Event::error()
{
	Error_Handler();
}

EventHandler::EventHandler(uint16_t _maxEvents)
	: maxEvents(_maxEvents)
{
	registeredEvents = new Event*[maxEvents];
	for (uint16_t i=0; i<maxEvents; ++i)
		registeredEvents[i] = nullptr;
}

EventHandler::~EventHandler()
{
	delete registeredEvents;
}

bool EventHandler::addEvent(Event* e)
{
	for (size_t i=0; i<maxEvents; ++i) {
		if (registeredEvents[i] == nullptr) {
			registeredEvents[i] = e;
			return true;
		}
	}
	return false;
}

void EventHandler::process()
{
	for (size_t i=0; i<maxEvents; ++i) {
		if (registeredEvents[i] == nullptr)
			break;
		else {
			if (registeredEvents[i]->isPending())
				registeredEvents[i]->process();
		}
	}
}
