
#ifndef RTC_USER_INIT
#define RTC_USER_INIT

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




#endif
