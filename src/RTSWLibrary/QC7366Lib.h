/*
 * QC7366Lib.h
 *
 * Created: 14-10-2022 15:00:12
 *  Author: Roel Smeets
 *
 * RASM, changed 16-04-2026:
 * changed QC_N_CHANNELS from 2 to 4 for use with external counters on
 * the SPI bus
 *
 */ 

// library & definitions for LS7366R quadrature counter

#ifndef QC7366LIB_H_
#define QC7366LIB_H_

#include "bits.h"

#define QC_N_CHANNELS	4
#define QC_MAX_CHANNEL	(QC_N_CHANNELS - 1)


// register definitions, bits 5..3

#define REG_MDR0	(0x1 << 3)
#define REG_MDR1	(0x2 << 3) 
#define REG_DTR		(0x3 << 3)
#define REG_CNTR	(0x4 << 3)
#define REG_OTR		(0x5 << 3)
#define REG_STR		(0x6 << 3)

// opcode definitions, bits 7..6

#define OP_CLR		(0x0 << 6)		// clear register
#define OP_READ		(0x1 << 6)		// read register
#define OP_WRITE	(0x2 << 6)		// write register
#define OP_LOAD		(0x3 << 6)		// load register

// clear registers

#define CLR_MDR0	(OP_CLR | REG_MDR0)
#define CLR_MDR1	(OP_CLR | REG_MDR1)
#define CLR_CNTR	(OP_CLR | REG_CNTR)
#define CLR_STR		(OP_CLR | REG_STR)

// read registers

#define READ_MDR0	(OP_READ | REG_MDR0)
#define READ_MDR1	(OP_READ | REG_MDR1)
#define READ_CNTR	(OP_READ | REG_CNTR)
#define READ_OTR	(OP_READ | REG_OTR)
#define READ_STR	(OP_READ | REG_STR)

// write registers

#define WRITE_MDR0	(OP_WRITE | REG_MDR0)
#define WRITE_MDR1	(OP_WRITE | REG_MDR1)
#define WRITE_DTR	(OP_WRITE | REG_DTR)

// load registers

#define LOAD_CNTR	(OP_LOAD | REG_CNTR)
#define LOAD_OTR	(OP_LOAD | REG_OTR)


// MDR0 quadrature count mode definitions, bits 1..0

#define MODE_NON_QC		0x00
#define MODE_QC_1		0x01
#define MODE_QC_2		0x02
#define MODE_QC_4		0x03

// MDR0 count mode definitions, bits 3..2

#define MODE_FREERUNNING	(0x0 << 2)
#define MODE_SINGLECYCLE	(0x1 << 2)
#define MODE_RANGELIMIT		(0x2 << 2)
#define MODE_MODULO_N		(0x3 << 2)

// MDR0 index mode definitions, bits 5..4

#define INDEX_DISABLE		(0x0 << 4)
#define INDEX_LOADCNTR		(0x1 << 4)
#define INDEX_RESETCNTR		(0x2 << 4)
#define INDEX_LOADOTR		(0x3 << 4)

#define INDEX_ASYNC			(0x0 << 6)
#define INDEX_SYNC			(0x1 << 6)

#define FILTERCLOCK_DIV_1	(0x0 << 7)
#define FILTERCLOCK_DIV_2	(0x1 << 7)


// MDR1 counter mode definitions, bits 1..0

#define CNTMODE_4			0x0		// 4 byte counter
#define CNTMODE_3			0x1		// 3 byte counter
#define CNTMODE_2			0x2		// 2 byte counter
#define CNTMODE_1			0x3		// 1 byte counter

// MDR1 counter control definitions, bit 2

#define CNT_DISABLE			(0x1 << 2)

// MDR1 flag bit definitions, bits 7..4

#define FLAG_ON_IDX			BIT_4
#define FLAG_ON_CMP			BIT_5
#define FLAG_ON_BW			BIT_6
#define FLAG_ON_CY			BIT_7
	

// status bits for STR (Status Register)

#define CY_BIT				BIT_7
#define BW_BIT				BIT_6
#define CMP_BIT				BIT_5
#define IDX_BIT				BIT_4
#define CEN_BIT				BIT_3
#define PLS_BIT				BIT_2
#define CNTUP_BIT			BIT_1
#define NEG_BIT				BIT_0



typedef enum 
{
	QC_MODE_REGISTER_0,
	QC_MODE_REGISTER_1,
} mode_register_t;

///////////////////////////////////////////////////////////////////////////////
// function prototypes

void qc_SelectChannel(uint8_t qcChannel);
void qc_DeselectChannel(uint8_t qcChannel);
void qc_SendCommand(uint8_t channel, uint8_t commandByte);

void	qc_Init(void);

void	qc_WriteModeRegister(uint8_t channel, mode_register_t modeRegister, uint8_t valueMDR);
void	qc_ClearModeRegister(uint8_t channel, uint8_t MDR0or1);
uint8_t qc_ReadModeRegister(uint8_t channel, uint8_t MDR0or1);

void	qc_ClearCountRegister(uint8_t channel);
int32_t qc_ReadCountRegister(uint8_t channel);

void	qc_ClearStatusRegister(uint8_t channel);
uint8_t qc_ReadStatusRegister(uint8_t channel);

void	qc_WriteDataRegister(uint8_t channel, int32_t dtrValue);
void	qc_TransferDataRegisterToCountRegister(uint8_t channel);
void	qc_TranferCountRegisterToOutputRegister(uint8_t channel);
int32_t qc_ReadOutputRegister(uint8_t channel);

void	qc_EnableCounter(uint8_t channel);
void	qc_DisableCounter(uint8_t channel);

bool	qc_IsIndexSet(uint8_t channel);

#endif /* QC7366LIB_H_ */