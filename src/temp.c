/*
 * temp.c
 *
 * Created: 08/05/2025
 */
#include "temp.h"

///////////////////////////////////////////////////////////////////////////////
// system includes
#include <asf.h>

///////////////////////////////////////////////////////////////////////////////
// FreeRTOS includes

#include "vPrintString.h"

///////////////////////////////////////////////////////////////////////////////
// FOR 1KHZ 1ms clock check!!
static uint32_t runLoopCpuHz = 0U;
static uint32_t runLoopCyclesPerMs = 0U;
static uint32_t runLoopStartCycles = 0U;
static uint32_t runLoopLastCycles = 0U;
static uint32_t runLoopMaxCycles = 0U;
static uint32_t runLoopPrintRefCycles = 0U;
static uint32_t runLoopOverruns = 0U;

///////////////////////////////////////////////////////////////////////////////
// void RunningLoopTimer_Init(void)
/*
 * Initialiseert de DWT-cycleteller voor timingcontrole van de 1 kHz loop.
 * Invoer: geen.
 * Uitvoer: geen returnwaarde; interne timerreferenties worden ingesteld.
 */
void RunningLoopTimer_Init(void)
{
	runLoopCpuHz = sysclk_get_cpu_hz();
	runLoopCyclesPerMs = runLoopCpuHz / 1000U;

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0U;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	runLoopPrintRefCycles = DWT->CYCCNT;
}


///////////////////////////////////////////////////////////////////////////////
// void RunningLoopTimer_ResetWindow(void)
/*
 * Wist de meetwaarden van het huidige timingvenster.
 * Invoer: geen.
 * Uitvoer: geen returnwaarde; maximale looptijd en overruns worden gereset.
 */
void RunningLoopTimer_ResetWindow(void)
{
	runLoopLastCycles = 0U;
	runLoopMaxCycles = 0U;
	runLoopOverruns = 0U;
	runLoopPrintRefCycles = DWT->CYCCNT;
}

///////////////////////////////////////////////////////////////////////////////
// void RunningLoopTimer_Begin(void)
/*
 * Markeer het begin van een control-loop meting.
 * Invoer: geen.
 * Uitvoer: geen returnwaarde; de huidige DWT-cycleteller wordt opgeslagen.
 */
void RunningLoopTimer_Begin(void)
{
	runLoopStartCycles = DWT->CYCCNT;
}

///////////////////////////////////////////////////////////////////////////////
// void RunningLoopTimer_End(void)
/*
 * Sluit een control-loop meting af en print periodiek de maximale looptijd.
 * Invoer: geen.
 * Uitvoer: geen returnwaarde; meetstatistieken en overrun-teller worden bijgewerkt.
 */
void RunningLoopTimer_End(void)
{
	uint32_t nowCycles;

	runLoopLastCycles = DWT->CYCCNT - runLoopStartCycles;

	if (runLoopLastCycles > runLoopMaxCycles)
	{
		runLoopMaxCycles = runLoopLastCycles;
	}

	if (runLoopLastCycles >= runLoopCyclesPerMs)
	{
		runLoopOverruns++;
	}

	nowCycles = DWT->CYCCNT;

	if ((uint32_t)(nowCycles - runLoopPrintRefCycles) >= runLoopCpuHz)
	{
		uint32_t max_us  = (uint32_t)(((uint64_t)runLoopMaxCycles  * 1000000U) / runLoopCpuHz);

		vPrintString("> RUN loop: max=%lu.%03lu ms, overruns=%lu\n",
		(unsigned long)(max_us / 1000U),
		(unsigned long)(max_us % 1000U),
		(unsigned long)runLoopOverruns);

		runLoopPrintRefCycles = nowCycles;
		runLoopMaxCycles = 0U;
		runLoopOverruns = 0U;
	}
}
