/*
 * twi.c
 *
 * Interrupt driven TWI (I2C) master driver for XMEGA devices
 */ 

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include <stdbool.h>
#include <stdio.h>

#include "twi.h"


#define	TWI_ERROR_COUNTER				// define to count errors in
#define TWI_IDLE_TIMEOUT_MS		10		// 0 = immediate fail if bus not idle

#define TWI						TWIC
#define TWI_MASTER_vect			TWIC_TWIM_vect
#define TWI_SLAVE_vect			TWIC_TWIS_vect
#define	TWI_PR_ENABLE			do { PR.PRPC &= ~PR_TWI_bm; } while(0)

#define TWI_BAUD_REG			75		// 100KHz, (Fclk/(2*Ftwi)) -5
#define TWI_MASTER_INTLVL_gc	TWI_MASTER_INTLVL_MED_gc


enum TRANSACTION_TYPES_enum
{
	TWI_TRANSACTION_IDLE,
	TWI_TRANSACTION_WRITE_REG,
	TWI_TRANSACTION_READ_REG,
	TWI_TRANSACTION_READ,
	TWI_TRANSACTION_PROBE,
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


volatile uint8_t transaction_type_AT = TWI_TRANSACTION_IDLE;
volatile uint8_t transaction_result_AT = TWI_RESULT_PENDING;
volatile uint8_t state_AT = TWI_STATE_REGISTER;
volatile uint8_t device_address_AT = 0;
volatile uint8_t reg_address_AT = 0;
volatile uint8_t data_bytes_AT = 0;
volatile uint8_t *data_buffer_AT = NULL;
void (* volatile callback_func_AT)(bool success);

#ifdef TWI_ERROR_COUNTER
volatile uint32_t TWI_fault_counter_AT = 0;
#endif


/**************************************************************************************************
* Set up the TWI peripheral in Master mode
*/
void TWI_init(void)
{
	TWI_PR_ENABLE;
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
					{
						TWI.MASTER.DATA = *data_buffer_AT++;
						data_bytes_AT--;
					}
					else
					{
						TWI.MASTER.STATUS = TWI_MASTER_RIF_bm | TWI_MASTER_WIF_bm | TWI_MASTER_BUSSTATE_IDLE_gc;
						transaction_type_AT = TWI_TRANSACTION_IDLE;
						transaction_result_AT = TWI_RESULT_OK;
						if (callback_func_AT != NULL)
							((void(*)(bool))callback_func_AT)(true);
					}
					break;
				
				default:
					goto reset;
			}
			break;
		
		// read register
		case TWI_TRANSACTION_READ_REG:
			if ((!(TWI.MASTER.STATUS & TWI_MASTER_WIF_bm)) ||
				(TWI.MASTER.STATUS & (TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm | TWI_MASTER_RXACK_bm)))
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
			break;

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
						*data_buffer_AT++ = TWI.MASTER.DATA;
						TWI.MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;		// send NACK
						transaction_type_AT = TWI_TRANSACTION_IDLE;
						transaction_result_AT = TWI_RESULT_OK;
						if (callback_func_AT != NULL)
							((void(*)(bool))callback_func_AT)(true);
					}
					break;
				
				default:
					goto reset;
			}
			
		// probe to see if a device is there
		case TWI_TRANSACTION_PROBE:
			if ((!(TWI.MASTER.STATUS & TWI_MASTER_RIF_bm)) ||
				(TWI.MASTER.STATUS & (TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm)))
				goto reset;
			
			TWI.MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;	// send NACK
			transaction_type_AT = TWI_TRANSACTION_IDLE;
			transaction_result_AT = TWI_RESULT_OK;
			if (callback_func_AT != NULL)
				((void(*)(bool))callback_func_AT)(true);
			break;
	}
	return;
	
reset:
	TWI.MASTER.STATUS = TWI_MASTER_RIF_bm | TWI_MASTER_WIF_bm | TWI_MASTER_BUSSTATE_IDLE_gc;
	transaction_type_AT = TWI_TRANSACTION_IDLE;
	transaction_result_AT = TWI_RESULT_FAILED;
	TWI_fault_counter_AT++;
	if (callback_func_AT != NULL)
		((void(*)(bool))callback_func_AT)(false);
}

/**************************************************************************************************
* Write a register. Callback can be null if not used.
*/
bool TWI_write_reg(uint8_t address, uint8_t reg, const void *buffer, uint8_t buffer_size, void (*callback)(bool success))
{
	// wait for bus to become idle
	uint8_t	i = 0;
	while (transaction_type_AT != TWI_TRANSACTION_IDLE)
	{
		if (i >= TWI_IDLE_TIMEOUT_MS)
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
	callback_func_AT = callback;
	TWI.MASTER.CTRLA |= TWI_MASTER_INTLVL_gc;

	// start write
	TWI.MASTER.ADDR = address << 1;		// send start and address and start interrupt state machine
	return true;
}

/**************************************************************************************************
* Read a register from TWI device. Callback can be null if not used.
*/
bool TWI_read_reg(uint8_t address, uint8_t reg, void *buffer, uint8_t buffer_size, void (*callback)(bool success))
{
	// wait for bus to become idle
	uint8_t	i = 0;
	while (transaction_type_AT != TWI_TRANSACTION_IDLE)
	{
		if (i >= TWI_IDLE_TIMEOUT_MS)
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
	callback_func_AT = callback;
	TWI.MASTER.CTRLA |= TWI_MASTER_INTLVL_gc;

	TWI.MASTER.ADDR = address << 1;		// sends start and address, starts interrupt state machine
	return true;
}

/**************************************************************************************************
* Read from TWI device. Callback can be null if not used.
*/
bool TWI_read(uint8_t address, void *buffer, uint8_t buffer_size, void (*callback)(bool success))
{
	// wait for bus to become idle
	uint8_t	i = 0;
	while (transaction_type_AT != TWI_TRANSACTION_IDLE)
	{
		if (i >= TWI_IDLE_TIMEOUT_MS)
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
	callback_func_AT = callback;
	TWI.MASTER.CTRLA |= TWI_MASTER_INTLVL_gc;

	TWI.MASTER.ADDR = (address << 1) | TWI_READ_bm;		// sends start and address
	return true;
}

/**************************************************************************************************
* Probe a TWI device by doing a dummy start read and looking for an ACK.
*/
bool TWI_probe(uint8_t address, void (*callback)(bool success))
{
	// wait for bus to become idle
	uint8_t	i = 0;
	while (transaction_type_AT != TWI_TRANSACTION_IDLE)
	{
		if (i >= TWI_IDLE_TIMEOUT_MS)
			return false;
		_delay_ms(1);
		i++;
	}

	// start read
	TWI.MASTER.CTRLA &= ~(TWI_MASTER_INTLVL_gm);		// make updates atomic
	transaction_type_AT = TWI_TRANSACTION_PROBE;
	transaction_result_AT = TWI_RESULT_PENDING;
	state_AT = TWI_STATE_DATA;
	device_address_AT = address;
	callback_func_AT = callback;
	TWI.MASTER.CTRLA |= TWI_MASTER_INTLVL_gc;

	TWI.MASTER.ADDR = (address << 1) | TWI_READ_bm;		// sends start and address
	return true;
}

/**************************************************************************************************
* Wait for TWI read/write to complete. Returns true if action was successful, false on failure.
*/
bool TWI_wait_for_completion(void)
{
	while (transaction_result_AT == TWI_RESULT_PENDING)
		;
	return transaction_result_AT == TWI_RESULT_OK ? true : false;
}

/**************************************************************************************************
* Wait for TWI read/write to complete. Returns true if action was successful, false on failure.
*/
bool TWI_wait_for_completion_sleep(void)
{
	while (transaction_result_AT == TWI_RESULT_PENDING)
		sleep_cpu();
	return transaction_result_AT == TWI_RESULT_OK ? true : false;
}

/**************************************************************************************************
* Returns true if last transaction completed successfully, false on error or if still in
* progress.
*/
bool TWI_was_successful(void)
{
	return transaction_result_AT == TWI_RESULT_OK ? true : false;
}

/**************************************************************************************************
* Returns true bus is free.
*/
bool TWI_is_bus_free(void)
{
	return transaction_type_AT == TWI_TRANSACTION_IDLE;
}

/**************************************************************************************************
* Resets the bus, ending any transactions in progress.
*/
void TWI_reset(void)
{
	TWI.MASTER.CTRLA = 0;
	TWI.MASTER.CTRLA = TWI_MASTER_INTLVL_gc | TWI_MASTER_RIEN_bm | TWI_MASTER_WIEN_bm | TWI_MASTER_ENABLE_bm;
	transaction_type_AT = TWI_TRANSACTION_IDLE;
}

/**************************************************************************************************
 * Scan for TWI devices, output to terminal
 */
void TWI_scan(void)
{
	puts_P(PSTR("Start I2C scan...\r"));

	for (uint8_t i = 1; i < 127; i++)
	{
		if (TWI_probe(i, NULL))
		{
			if (TWI_wait_for_completion())
				printf_P(PSTR("Device found at 0x%02X\r\n"), i);
		}
	}
	
	puts_P(PSTR("Scan complete.\r\n"));
}
