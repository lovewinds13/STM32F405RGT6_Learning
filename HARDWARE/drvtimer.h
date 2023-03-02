#ifndef __DRVTIMER_H__
#define __DRVTIMER_H__

#include "sys.h"

extern void timer_config(uint8_t timer_no, uint16_t timer_arr, uint16_t timer_psc);
extern void timer_init(uint8_t timer_no);

	
#endif	//__DRVTIMER_H__
