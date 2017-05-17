/*
 * twi.c
 *
 * TWI (I2C) master driver for XMEGA devices
 */ 

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <stdbool.h>
#include <stdio.h>

#include "twi.h"


enum TRANSACTION_TYPES_enum
{
	TWI_TRANSACTION_IDLE,
	TWI_TRANSACTION_WRITE_REG,
	TWI_TRANSACTION_READ_REG,
	TWI_TRANSACTION_READ
};

enum TRANSACTION_RESULTS_enum
{
	TWI_RESULT_PENDING,
	TWI_RESULT_FAILED,
	TWI_RESULT_OK
};

enum STATE_MACHINE_enum
{
	TWI_STATE_REGISTER,
	TWI_STATE_RESTART,
	TWI_STATE_DATA
};

#define TWI_READ_bm				(1<<0)



volatile uint8_t	transaction_type_AT = TWI_TRANSACTION_IDLE;
volatile uint8_t transaction_result_AT = TWI_RESULT_PENDING;
volatile uint8_t state_AT = TWI_STATE_REGISTER;
volatile uint8_t	device_address_AT = 0;
volatile uint8_t	reg_address_AT = 0;
volatile uint8_t data_bytes_AT = 0;
volatile uint8_t	*data_buffer_AT = NULL;



#ifndef TWI_INTERRUPT_DRIVEN


/**************************************************************************************************
 * Set up the TWI peripheral in Master mode
 */
void TWI_init(void)
{
	PR.PRPC	&= ~PR_TWI_bm;
	TWI.CTRL = 0;	// SDA hold time off
	
	TWI.MASTER.CTRLB = TWI_MASTER_TIMEOUT_200US_gc;
	TWI.MASTER.CTRLC = 0;
	TWI.MASTER.BAUD = TWI_BAUD_REG;
	TWI.MASTER.CTRLA = TWI_MASTER_ENABLE_bm;
	TWI.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;
}

/**************************************************************************************************
 * Write a register
 */
bool TWI_write_reg(uint8_t address, uint8_t reg, const uint8_t *buffer, uint8_t buffer_size)
{
	__label__ failure;
	uint8_t	i;

	// wait for bus to become idle
	i = 0;
	while ((TWI.MASTER.STATUS & TWI_MASTER_BUSSTATE_gm) != TWI_MASTER_BUSSTATE_IDLE_gc)
	{
		if (i > TWI_IDLE_TIMEOUT_MS)
			goto failure;
		_delay_ms(1);
		i++;
	}

	// start write
	TWI.MASTER.ADDR = address << 1;		// sends start and address
	while ((TWI.MASTER.STATUS & TWI_MASTER_WIF_bm) == 0);
	if (TWI.MASTER.STATUS & (TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm))
		goto failure;

	// register address
	TWI.MASTER.DATA = reg;
	while ((TWI.MASTER.STATUS & TWI_MASTER_WIF_bm) == 0);
	if (TWI.MASTER.STATUS & (TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm))
		goto failure;

	// data bytes
	for (i = 0; i < buffer_size; i++)
	{
		TWI.MASTER.DATA = buffer[i];
		while ((TWI.MASTER.STATUS & TWI_MASTER_WIF_bm) == 0);
		if (TWI.MASTER.STATUS & (TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm))
			goto failure;
	}

	// transaction complete or failed
	return true;
failure:
	TWI.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
	return false;
}

/**************************************************************************************************
 * Read a register from TWI device
 */
bool TWI_read_reg(uint8_t address, uint8_t reg, uint8_t *buffer, uint8_t buffer_size)
{
	__label__ failure;
	uint8_t	i;

	// wait for bus to become idle
	i = 0;
	while ((TWI.MASTER.STATUS & TWI_MASTER_BUSSTATE_gm) != TWI_MASTER_BUSSTATE_IDLE_gc)
	{
		if (i > TWI_IDLE_TIMEOUT_MS)
			goto failure;
		_delay_ms(1);
		i++;
	}

	// start write
	TWI.MASTER.ADDR = address << 1;		// sends start and address
	while ((TWI.MASTER.STATUS & TWI_MASTER_WIF_bm) == 0);
	if (TWI.MASTER.STATUS & (TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm))
		goto failure;

	// register address
	TWI.MASTER.DATA = reg;
	while ((TWI.MASTER.STATUS & TWI_MASTER_WIF_bm) == 0);
	if (TWI.MASTER.STATUS & (TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm))
		goto failure;

	// (re)start read
	TWI.MASTER.ADDR = (address << 1) | TWI_READ_bm;		// sends repeated start and address

	// read requested number of bytes
	uint8_t	timeout = 200;	// double timeout to account for read setup byte
	while (i)
	{
		while ((TWI.MASTER.STATUS & (TWI_MASTER_RIF_bm | TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm)) == 0)
		{
			timeout--;
			if (timeout == 0)
				goto failure;
			_delay_us(1);
		}
		timeout = 100;

		*buffer++ = TWI.MASTER.DATA;

		if (TWI.MASTER.STATUS & (TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm))
			goto failure;
		i--;
		if (i != 0)
			TWI.MASTER.CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;							// send ACK
		else
			TWI.MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;		// send NACK
	}

	// transaction complete or failed
	return true;
failure:
	TWI.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
	return false;
}

/**************************************************************************************************
 * Read from TWI device
 */
bool TWI_read(uint8_t address, uint8_t *buffer, uint8_t buffer_size)
{
	__label__ failure;
	uint8_t	i;

	// wait for bus to become idle
	i = 0;
	while ((TWI.MASTER.STATUS & TWI_MASTER_BUSSTATE_gm) != TWI_MASTER_BUSSTATE_IDLE_gc)
	{
		if (i > 10)
			goto failure;
		_delay_ms(1);
		i++;
	}

	// start read
	TWI.MASTER.ADDR = (address << 1) | TWI_READ_bm;		// sends start and address

	// read requested number of bytes
	i = buffer_size;
	uint8_t	timeout = 200;	// double timeout to account for read setup byte
	while (i)
	{
		while ((TWI.MASTER.STATUS & (TWI_MASTER_RIF_bm | TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm)) == 0)
		{
			timeout--;
			if (timeout == 0)
				goto failure;
			_delay_us(1);
		}
		timeout = 100;

		*buffer++ = TWI.MASTER.DATA;

		if (TWI.MASTER.STATUS & (TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm))
			goto failure;
		i--;
		if (i != 0)
			TWI.MASTER.CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;							// send ACK
		else
			TWI.MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;		// send NACK
	}

	// transaction complete or failed
	return true;
failure:
	TWI.MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;

	return false;
}


#endif

#ifdef TWI_INTERRUPT_DRIVEN


/**************************************************************************************************
* Set up the TWI peripheral in Master mode
*/
void TWI_init(void)
{
	PR.PRPC	&= ~PR_TWI_bm;
	TWI.CTRL = 0;	// SDA hold time off
	
	TWI.MASTER.CTRLB = TWI_MASTER_TIMEOUT_200US_gc;
	TWI.MASTER.CTRLC = 0;
	TWI.MASTER.BAUD = TWI_BAUD_REG;
	TWI.MASTER.CTRLA = TWI_MASTER_INTLVL_gc | TWI_MASTER_RIEN_bm | TWI_MASTER_WIEN_bm | TWI_MASTER_ENABLE_bm;
	TWI.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;
}

/**************************************************************************************************
* Master mode interrupt handler
*/
ISR(TWI_MASTER_vect)
{
	__label__ reset;
	
	switch(transaction_type_AT)
	{
		case TWI_TRANSACTION_IDLE:
			goto reset;
			break;
		
		// write register
		case TWI_TRANSACTION_WRITE_REG:
			if ((!(TWI.MASTER.STATUS & TWI_MASTER_WIF_bm)) ||
				(TWI.MASTER.STATUS & (TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm)))
				goto reset;
			
			switch(state_AT)
			{
				case TWI_STATE_REGISTER:
					TWI.MASTER.DATA = reg_address_AT;
					state_AT = TWI_STATE_DATA;
					break;
				
				case TWI_STATE_DATA:
					if (data_bytes_AT)
						TWI.MASTER.DATA = *data_buffer_AT++;
					else
						transaction_result_AT = TWI_RESULT_OK;
					break;
				
				default:
					goto reset;
			}
		
		// read register
		case TWI_TRANSACTION_READ_REG:
			if ((!(TWI.MASTER.STATUS & TWI_MASTER_WIF_bm)) ||
				(TWI.MASTER.STATUS & (TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm)))
				goto reset;
			
			switch(state_AT)
			{
				case TWI_STATE_REGISTER:
					TWI.MASTER.DATA = reg_address_AT;
					state_AT = TWI_STATE_RESTART;
					break;
				
				case TWI_STATE_RESTART:
					TWI.MASTER.ADDR = (device_address_AT << 1) | TWI_READ_bm;		// sends repeated start and address
					state_AT = TWI_STATE_DATA;
					transaction_type_AT = TWI_TRANSACTION_READ;						// common code
					break;

				default:
					goto reset;
			}

		// read
		case TWI_TRANSACTION_READ:
			if ((!(TWI.MASTER.STATUS & TWI_MASTER_RIF_bm)) ||
				(TWI.MASTER.STATUS & (TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm)))
				goto reset;
			
			switch(state_AT)
			{
				case TWI_STATE_DATA:
					data_bytes_AT--;
					if (data_bytes_AT != 0)
						TWI.MASTER.CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;							// send ACK
					else
					{
						TWI.MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;		// send NACK
						transaction_result_AT = TWI_RESULT_OK;
					}
					*data_buffer_AT++ = TWI.MASTER.DATA;
					break;
				
				default:
					goto reset;
			}
			
	}
	
reset:
	TWI.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;
	transaction_type_AT = TWI_TRANSACTION_IDLE;
	transaction_result_AT = TWI_RESULT_FAILED;
}

/**************************************************************************************************
* Write a register
*/
bool TWI_write_reg(uint8_t address, uint8_t reg, const uint8_t *buffer, uint8_t buffer_size)
{
	uint8_t	i;

	// wait for bus to become idle
	i = 0;
	while ((TWI.MASTER.STATUS & TWI_MASTER_BUSSTATE_gm) != TWI_MASTER_BUSSTATE_IDLE_gc)
	{
		if (i > TWI_IDLE_TIMEOUT_MS)
			return false;
		_delay_ms(1);
		i++;
	}

	// start write
	TWI.MASTER.CTRLA &= ~(TWI_MASTER_INTLVL_gm);		// make updates atomic
	transaction_type_AT = TWI_TRANSACTION_WRITE_REG;
	transaction_result_AT = TWI_RESULT_PENDING;
	state_AT = TWI_STATE_REGISTER;
	device_address_AT = address;
	reg_address_AT = reg;
	data_bytes_AT = buffer_size;
	data_buffer_AT = (uint8_t *)buffer;
	TWI.MASTER.CTRLA |= TWI_MASTER_INTLVL_gc;

	// start write
	TWI.MASTER.ADDR = address << 1;		// sends start and address and start interrupt state machine
	return true;
}

/**************************************************************************************************
* Read a register from TWI device
*/
bool TWI_read_reg(uint8_t address, uint8_t reg, uint8_t *buffer, uint8_t buffer_size)
{
	uint8_t	i;
				
	// wait for bus to become idle
	i = 0;
	while ((TWI.MASTER.STATUS & TWI_MASTER_BUSSTATE_gm) != TWI_MASTER_BUSSTATE_IDLE_gc)
	{
		if (i > TWI_IDLE_TIMEOUT_MS)
			return false;
		_delay_ms(1);
		i++;
	}

	// start read
	TWI.MASTER.CTRLA &= ~(TWI_MASTER_INTLVL_gm);		// make updates atomic
	transaction_type_AT = TWI_TRANSACTION_READ_REG;
	transaction_result_AT = TWI_RESULT_PENDING;
	state_AT = TWI_STATE_REGISTER;
	device_address_AT = address;
	reg_address_AT = reg;
	data_bytes_AT = buffer_size;
	data_buffer_AT = buffer;
	TWI.MASTER.CTRLA |= TWI_MASTER_INTLVL_gc;

	TWI.MASTER.ADDR = address << 1;		// sends start and address, starts interrupt state machine
	return true;
}

/**************************************************************************************************
* Read from TWI device
*/
bool TWI_read(uint8_t address, uint8_t *buffer, uint8_t buffer_size)
{
	uint8_t	i;

	// wait for bus to become idle
	i = 0;
	while ((TWI.MASTER.STATUS & TWI_MASTER_BUSSTATE_gm) != TWI_MASTER_BUSSTATE_IDLE_gc)
	{
		if (i > 10)
			return false;
		_delay_ms(1);
		i++;
	}

	// start read
	TWI.MASTER.CTRLA &= ~(TWI_MASTER_INTLVL_gm);		// make updates atomic
	transaction_type_AT = TWI_TRANSACTION_READ;
	transaction_result_AT = TWI_RESULT_PENDING;
	state_AT = TWI_STATE_DATA;
	device_address_AT = address;
	reg_address_AT = 0;
	data_bytes_AT = buffer_size;
	data_buffer_AT = buffer;
	TWI.MASTER.CTRLA |= TWI_MASTER_INTLVL_gc;

	TWI.MASTER.ADDR = (address << 1) | TWI_READ_bm;		// sends start and address
	return(true);
}


#endif


/**************************************************************************************************
 * Scan for TWI devices, output to terminal
 */
void TWI_scan(void)
{
	uint8_t	i;
	
	for (i = 1; i < 127; i++)
	{
		if (TWI_read(i, NULL, 0))
			printf_P(PSTR("Device found at 0x%02X\r\n"), i);
	}
	
	puts_P(PSTR("Scan complete.\r\n"));
}
