/*
 * twi.h
 *
 */ 


#ifndef TWI_H_
#define TWI_H_

#include <stdbool.h>


extern volatile uint32_t TWI_fault_counter_AT;


extern void TWI_init(void);
extern bool TWI_write_reg(uint8_t address, uint8_t reg, const void *buffer, uint8_t buffer_size, void (*callback)(bool success));
extern bool TWI_read_reg(uint8_t address, uint8_t reg, void *buffer, uint8_t buffer_size, void (*callback)(bool success));
extern bool TWI_read(uint8_t address, void *buffer, uint8_t buffer_size, void (*callback)(bool success));
extern bool TWI_probe(uint8_t address, void (*callback)(bool success));
extern bool TWI_wait_for_completion(void);
extern bool TWI_wait_for_completion_sleep(void);
extern bool TWI_was_successful(void);
extern bool TWI_is_bus_free(void);
extern void TWI_reset(void);
extern void TWI_scan(void);



#endif /* TWI_H_ */