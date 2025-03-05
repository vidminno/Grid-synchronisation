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
 
 
#ifndef GRID_FREQUENCY_H
#define GRID_FREQUENCY_H

//System-Includes
#include "stm32h7xx.h"
#include "main.h"

// User-Includes
#include "timer_base.h"
#include "event.h"

constexpr float PiF = static_cast<float>(M_PI);


struct GridSyncTim : public TimBase {
	GridSyncTim(TIM_TypeDef* instance, uint32_t _clkFrequ, GPIO_TypeDef** _gpiox, uint32_t* _px, uint32_t* _alternate)
		: TimBase(instance, _clkFrequ, _gpiox, _px, _alternate), tickRate(0.0f)
	{}

	float tickRate;
};

// Set up timer for the grid frequency and phase determination
void gridSyncTimInit(GridSyncTim& tim);


class GridSync {
public:
	GridSync(GridSyncTim& _gridTim, uint32_t _cpuFrequ)
	 	 : gridTim(_gridTim), ticksZeroLast(0), ticksZero(0), posHalfWave(false), sumTicksError(0.0f), lastTicksError(0.0f), ticksBetweenZerosNext(0.0f), updatePllLast(0), cpuFrequ(_cpuFrequ), firstRun(false)
	 {
		setPID(0.8315f, 0.006f, 0);
		frequRatio = (uint32_t)(cpuFrequ / gridTim.tickRate + 0.5f);
		ticksBetweenZerosNext = 1800000;  // Testing: Start at 50 Hz even if no zero crossings were detected
	 }

	// kP	P-Factor [1]
	// kI	I-Factor [1/T]
	// kD	D-Factor [T]
	void setPID(float _kP, float _kI, float _kD);

	// ticks		CPU-Counter (Is increased in the CPU clock)
	// cnt			Timer, which can have a clock rate that differs from the CPU clock
	// dir          - -> + dir=1     + -> - dir=0
	void updatePLL(uint32_t ticksO0, uint32_t cntDiffO0, uint32_t ticksO1, uint32_t cntDiffO1, bool dir);

	// Get determined grid frequency
	float gridFrequ();

	// Calculate current grid sin value. It is possible to add a phase and an dt (To look into the future or back)
	float sinGrid(float dt=0.0f, float phi=0.0f);

	// Calculate current grid phase related to sine value (0° to 359.99°). It is possible to add a phase and an dt (To look into the future or back)
	float gridPhaseRad(float dt=0.0f, float phi=0.0f);

	uint32_t gridTimARR() const { return gridTim.timInst->ARR; }

	// Norm the angle to (0° to 359.99°) (e.g. 360° -> 0 °)
	static inline float angleNormalize(float angle) { return angle - 2.0f * PiF * floor(angle/(2.0f*PiF)); }
protected:
	GridSyncTim& gridTim;
	uint32_t frequRatio;				// CPU-Counter and Timer can have a different clock rate
	uint32_t ticksZeroLast;
	uint32_t ticksZero;
	bool posHalfWave;

	float kP;							// PID-Parameter
	float kI;
	float kD;
	float sumTicksError;				// To build I-Term
	float lastTicksError;				// To build P-Term
	float ticksBetweenZerosNext;		// CPU-Ticks between two zero crossings
	uint32_t updatePllLast;				// To determine update-Intervall time

	float cpuFrequ;
	bool firstRun;
};


class EvGridSync : public Event {
public:
	EvGridSync(const char* _name, GridSync& _grid)
		: Event(_name), grid(_grid), cntDiffO0(0), appearTicksO0(0), edgeO0(false), cntDiffO1(0), appearTicksO1(0), edgeO1(false)
	{
		cntMax = grid.gridTimARR();
	}

	virtual ~EvGridSync() {}

	void evO0(uint32_t cntEv, uint32_t cntNow, bool edge);
	void evO1(uint32_t cntEv, uint32_t cntNow, bool edge);

	virtual void process() override;

protected:
	GridSync& grid;
	uint32_t cntMax;			// Maximum timer value (necessary for overflow safe calculation)
	uint32_t cntDiffO0;			// Timer ticks between event and ISR call for first optocoupler
	uint32_t appearTicksO0;		// CPU-Ticks for the first optocoupler at ISR call
	bool edgeO0;				// 0: N-Edge, 1: P-Edge
	uint32_t cntDiffO1;			// Timer ticks between event and ISR call for second optocoupler
	uint32_t appearTicksO1;		// CPU-Ticks for the second optocoupler at ISR call
	bool edgeO1;				// 0: N-Edge, 1: P-Edge
};


#endif
