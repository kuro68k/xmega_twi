/*
 * twi.h
 *
 */ 


#ifndef TWI_H_
#define TWI_H_


#define TWI						TWIC
#define TWI_MASTER_vect			TWIC_TWIM_vect
#define TWI_SLAVE_vect			TWIC_TWIS_vect

#define TWI_BAUD_REG			75		// 100KHz, (Fclk/(2*Ftwi)) -5
#define TWI_MASTER_INTLVL_gc	TWI_MASTER_INTLVL_MED_gc
#define TWI_IDLE_TIMEOUT_MS		10

//#define TWI_INTERRUPT_DRIVEN



extern void TWI_init(void);
extern bool TWI_write_reg(uint8_t address, uint8_t reg, const uint8_t *buffer, uint8_t buffer_size);
extern bool TWI_read_reg(uint8_t address, uint8_t reg, uint8_t *buffer, uint8_t buffer_size);
extern bool TWI_read(uint8_t address, uint8_t *buffer, uint8_t buffer_size);
extern void TWI_scan(void);



#endif /* TWI_H_ */