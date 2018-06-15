
#ifndef RTC_USER_INIT
#define RTC_USER_INIT

#include "stm32f4xx_hal.h"


typedef enum
{
	hoursInit=13,
	MinutesInit,
	SecondsInit,
	DayInit,
	MonthInit,
	YearInit,
	FinishSet
} State_t;



extern void SetSystemTime(void);
extern void InitializeAlarm(uint8_t hrs, uint8_t min);




#endif
