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
#include <math.h>

// User-Includes
#include "grid_frequency.h"


void gridSyncTimInit(GridSyncTim& tim)
{
	timDeactivate(tim);
	timInitPins(tim);

	constexpr uint32_t arr = 0xffff;	// max(uint16_t)
	uint32_t psc = 0;

	/* f              Counting frequency
	 * ftim           Timer frequency
	 * ftim = f / ((PSC+1)*(ARR+1))
	 * <=>
	 * PSC = f / ( ftim*(ARR+1)) - 1

	float ftim = 90.0f;				// has to be < 2*fgrid

	psc = ceil(tim.clkFrequ / (ftim*(arr+1UL)) -1.0f);
	assert(psc < 0xffff);
	assert(tim.timFrequ <= ftim);

	// Set prescaler, auto-reload-register
	tim.timInst->PSC = psc;
	tim.timInst->ARR = arr;
	 */

	// TIM is running with highest frequency
	tim.timInst->PSC = psc;
	tim.timInst->ARR = arr;

	float timFrequ = tim.clkFrequ / ((float)(psc+1UL)*(float)(arr+1UL));
	tim.tickRate = tim.clkFrequ / (float)(psc+1UL);

	// CKD[1:0] = 10: clk / 4
	//tim.inst->CR1 |= TIM_CR1_CKD_1;

	//  CC1IE[0] = 1: Capture/Compare 1 interrupt enable
	tim.timInst->DIER |= TIM_DIER_CC1IE;

	// CC1G[0] = 1: capture/compare event is generated on ch1
	tim.timInst->EGR |= TIM_EGR_CC1G;

	// CC1S[1:0] = 01: CC1 is input and mapped on TI1
	tim.timInst->CCMR1 |= TIM_CCMR1_CC1S_0;

	// IC1F[3:0] = 10: 4 clock cycle input filter
	tim.timInst->CCMR1 |= TIM_CCMR1_IC1F_1;

	// CC1P[0] = 1
	// CC1NP[0] = 1: Sensitive to both edges
	tim.timInst->CCER |= TIM_CCER_CC1P;
	tim.timInst->CCER |= TIM_CCER_CC1NP;

	// CC1E[0] = 1: Capture/Compare 1 output enable
	tim.timInst->CCER |= TIM_CCER_CC1E;

	// TISEL[3:0] = 0: TIM15_CH1 is input

	// And the same for CH2
	tim.timInst->DIER |= TIM_DIER_CC2IE;
	tim.timInst->EGR |= TIM_EGR_CC2G;
	tim.timInst->CCMR1 |= TIM_CCMR1_CC2S_0;
	tim.timInst->CCMR1 |= TIM_CCMR1_IC2F_1;
	tim.timInst->CCER |= TIM_CCER_CC2P;
	tim.timInst->CCER |= TIM_CCER_CC2NP;
	tim.timInst->CCER |= TIM_CCER_CC2E;

	tim.timInst->SR = 0;
	timActivate(tim);
}

void GridSync::setPID(float _kP, float _kI, float _kD)
{
	kP = _kP;
	kI = _kI;
	kD = _kD;
}

void GridSync::updatePLL(uint32_t ticksO0, uint32_t cntDiffO0, uint32_t ticksO1, uint32_t cntDiffO1, bool dir)
{
	// differences will stay correct through an uint32_t overflow

	// Correct the CPU time stamp by the time until the interrupt is processed
	ticksO0 = ticksO0 + cntDiffO0 * frequRatio;
	ticksO1 = ticksO1 + cntDiffO1 * frequRatio;

	//  - -> + dir=1
	if (dir) {
		posHalfWave = true;
		ticksZero = ticksO1 + (ticksO0 - ticksO1)/2;
	}
	// + -> - dir=0
	else {
		posHalfWave = false;
		ticksZero = ticksO0 + (ticksO1 - ticksO0)/2;
	}

	uint32_t ticksBetweenZerosNow = ticksZero - ticksZeroLast;
	ticksZeroLast = ticksZero;

	if (firstRun){
		ticksBetweenZerosNext = ticksBetweenZerosNow;
		firstRun = true;
	}
	else {
		/*
		 * Differential equation of parallel PID controller:
		 * 	y(t) = Kp*e + Ki*Int(e) + Kd*d/dt*e
		 * * Discrete time:
		 * 	yk = Kp*ek + Ki*Ta*Sum(e) + Kd/Ta*(ek-ek-1)
		 *  yk = pFactor*eDelta + iFactor*callIntervall*eDeltaSum + (eDelta-eDeltaOld)*dFactor/callIntervall;
		 */

		// ToDo: Implement fixed point arithmetic
		float ticksError = (float)ticksBetweenZerosNow - ticksBetweenZerosNext;
		float durationBetweenCalls = (float)(DWT->CYCCNT - updatePllLast) / cpuFrequ;
		updatePllLast = DWT->CYCCNT;

		// PID-Controller: P-Term
		float pTerm = ticksError*kP;

		// PID-Controller: I-Term. ToDo: Implement better discretisation method, e.g. the trapezoidal method
		sumTicksError = sumTicksError + ticksError*durationBetweenCalls;
		float iTerm = kI * sumTicksError;

		// PID-Controller: D-Term. ToDo: Implement better discretisation method, e.g. the trapezoidal method
		float dTerm = (ticksError - lastTicksError) * kD / durationBetweenCalls;
		lastTicksError = ticksError;

		// Sum terms
		ticksBetweenZerosNext = pTerm + iTerm + dTerm;
	}
}

float GridSync::gridFrequ()
{
	return 1.0f / (2.0f * ticksBetweenZerosNext / cpuFrequ);
}

float GridSync::sinGrid(float dt, float phi)
{
	return sin(gridPhaseRad(dt, phi));
}

float GridSync::gridPhaseRad(float dt, float phi)
{
	uint32_t ticksNow = DWT->CYCCNT;

	// w = 2*PI*f = 2*PI*1/T = PI/(0.5*T)    Ticks between two zero crossings corresponds to 0.5*t
	float w =  PiF / (float)ticksBetweenZerosNext;
	// Consider time since zero crossing. Add passed dt in CPU ticks
	float t = (float)(ticksNow - ticksZero) + dt * cpuFrequ;

	float angle;
	if (posHalfWave)
		angle = w*t + phi;
	else
		angle = w*t + phi + PiF;
	return angleNormalize(angle);
}


void EvGridSync::evO0(uint32_t cntEv, uint32_t cntNow, bool edge)
{
	appearTicksO0 = DWT->CYCCNT;
	appearTicks = appearTicksO0;

	// Consider overflow
	if (cntNow >= cntEv)
		cntDiffO0 = cntNow - cntEv;
	else
		cntDiffO0 = cntMax + cntNow - cntEv + 1;

	edgeO0 = edge;	// Pin is low => N-Edge

	if (!edge && edgeO1) {
		if (pending)
			error();
		pending = true;
	}
}


void EvGridSync::evO1(uint32_t cntEv, uint32_t cntNow, bool edge)
{
	appearTicksO1 = DWT->CYCCNT;
	appearTicks = appearTicksO1;

	// Consider overflow
	if (cntNow >= cntEv)
		cntDiffO1 = cntNow - cntEv;
	else
		cntDiffO1 = cntMax + cntNow - cntEv + 1;

	edgeO1 = edge;	// Pin is low => N-Edge

	if (!edge && edgeO0) {
		if (pending)
			error();
		pending = true;
	}
}

void EvGridSync::process()
{
	if (!edgeO0 && edgeO1)
		grid.updatePLL(appearTicksO0, cntDiffO0, appearTicksO1, cntDiffO1, true);
	else if (edgeO0 && !edgeO1)
		grid.updatePLL(appearTicksO0, cntDiffO0, appearTicksO1, cntDiffO1, false);
	else
		error();

	pending = false;
}
