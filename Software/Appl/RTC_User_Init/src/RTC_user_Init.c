#include "RTC_user_Init.h"
#include "stm32_topway_16x2.h"



static RTC_HandleTypeDef hrtc;
static State_t state = hoursInit;
static uint8_t hours = 0, minutes = 0,seconds,days = 1,months = 1, years = 18;

static void MX_RTC_UserInit(uint8_t hrs, uint8_t min, uint8_t sec, uint8_t day, uint8_t mon, uint16_t yrs);
static void SetHours(void);
static void SetMin(void);
static void SetSec(void);
static void SetDay(void);
static void SetMonth(void);
static void SetYear(void);



static void MX_RTC_UserInit(uint8_t hrs, uint8_t min, uint8_t sec, uint8_t day, uint8_t mon, uint16_t yrs)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 251;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = hrs;
  sTime.Minutes = min;
  sTime.Seconds = sec;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_SET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = mon;
  sDate.Date = day;
  sDate.Year = yrs;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

static void SetHours(void)
{
	if(!HAL_GPIO_ReadPin(KEYRIGHT_GPIO_Port, KEYRIGHT_Pin))
	{
		state = MinutesInit;
	}
	else if(!HAL_GPIO_ReadPin(KEYUP_GPIO_Port, KEYUP_Pin))
	{
		hours = (hours == 23)? 0 : hours+1;
	}
	else if(!HAL_GPIO_ReadPin(KEYDOWN_GPIO_Port, KEYDOWN_Pin))
	{
		hours = (hours == 0)? 23 : hours-1;
	}
	else
	{
		/*do nothing*/
	}
}

static void SetMin(void)
{
	if(!HAL_GPIO_ReadPin(KEYRIGHT_GPIO_Port, KEYRIGHT_Pin))
	{
		state = SecondsInit;
	}
	else if (!HAL_GPIO_ReadPin(KEYLEFT_GPIO_Port, KEYLEFT_Pin))
	{
		state = hoursInit;
	}
	else if(!HAL_GPIO_ReadPin(KEYUP_GPIO_Port, KEYUP_Pin))
	{
		minutes = (minutes == 59)? 0 : minutes+1;
	}
	else if(!HAL_GPIO_ReadPin(KEYDOWN_GPIO_Port, KEYDOWN_Pin))
	{
		minutes = (minutes == 0)? 59 : minutes-1;
	}
	else
	{
		/*do nothing*/
	}
}

static void SetSec(void)
{
	if(!HAL_GPIO_ReadPin(KEYRIGHT_GPIO_Port, KEYRIGHT_Pin))
	{
		state = DayInit;
	}
	else if (!HAL_GPIO_ReadPin(KEYLEFT_GPIO_Port, KEYLEFT_Pin))
	{
		state = MinutesInit;
	}
	else if(!HAL_GPIO_ReadPin(KEYUP_GPIO_Port, KEYUP_Pin))
	{
		seconds = (seconds == 59)? 0 : seconds+1;
	}
	else if(!HAL_GPIO_ReadPin(KEYDOWN_GPIO_Port, KEYDOWN_Pin))
	{
		seconds = (seconds == 0)? 59 : seconds-1;
	}
	else
	{
		/*do nothing*/
	}
}

static void SetDay(void)
{
	if(!HAL_GPIO_ReadPin(KEYRIGHT_GPIO_Port, KEYRIGHT_Pin))
	{
		state = MonthInit;
	}
	else if (!HAL_GPIO_ReadPin(KEYLEFT_GPIO_Port, KEYLEFT_Pin))
	{
		state = SecondsInit;
		LCD1602_clear();
	}
	else if(!HAL_GPIO_ReadPin(KEYUP_GPIO_Port, KEYUP_Pin))
	{
		days = (days == 31)? 1 : days+1;
	}
	else if(!HAL_GPIO_ReadPin(KEYDOWN_GPIO_Port, KEYDOWN_Pin))
	{
		days = (days == 1)? 31 : days-1;
	}
	else
	{
		/*do nothing*/
	}
}

static void SetMonth(void)
{
	if(!HAL_GPIO_ReadPin(KEYRIGHT_GPIO_Port, KEYRIGHT_Pin))
	{
		state = YearInit;
	}
	else if (!HAL_GPIO_ReadPin(KEYLEFT_GPIO_Port, KEYLEFT_Pin))
	{
		state = DayInit;
	}
	else if(!HAL_GPIO_ReadPin(KEYUP_GPIO_Port, KEYUP_Pin))
	{
		months = (months == 12)? 1 : months+1;
	}
	else if(!HAL_GPIO_ReadPin(KEYDOWN_GPIO_Port, KEYDOWN_Pin))
	{
		months = (months == 1)? 12 : months-1;
	}
	else
	{
		/*do nothing*/
	}
}

static void SetYear(void)
{
	if(!HAL_GPIO_ReadPin(KEYRIGHT_GPIO_Port, KEYRIGHT_Pin))
	{
		state = YearInit;
		MX_RTC_UserInit(hours, minutes,seconds,days,months, years);
		LCD1602_clear();
		state = FinishSet;
	}
	else if (!HAL_GPIO_ReadPin(KEYLEFT_GPIO_Port, KEYLEFT_Pin))
	{
		state = MonthInit;
	}
	else if(!HAL_GPIO_ReadPin(KEYUP_GPIO_Port, KEYUP_Pin))
	{
		years = (years == 18)? 50 : years+1;
	}
	else if(!HAL_GPIO_ReadPin(KEYDOWN_GPIO_Port, KEYDOWN_Pin))
	{
		years = (years == 50)? 18 : years-1;
	}
	else
	{
		/*do nothing*/
	}
}

void InitializeAlarm(uint8_t hrs, uint8_t min)
{
	RTC_AlarmTypeDef sAlarm;
	RTC_DateTypeDef sDate;
	RTC_TimeTypeDef sTime;

	HAL_RTC_GetDate(&hrtc,&sDate,RTC_FORMAT_BIN);
	HAL_RTC_GetTime(&hrtc, &sTime,RTC_FORMAT_BIN);

	if(
			(hrs <= sTime.Hours)
			&&
			(min <= sTime.Minutes)
	  )
	{
		if(
				(sDate.Month == 4)
				||
				(sDate.Month == 6)
				||
				(sDate.Month == 9)
				||
				(sDate.Month == 11)
		 )
		{
			sDate.Date = (sDate.Date + 1) % 30 ;
		}
		else if(sDate.Month == 2)
		{
			if((sDate.Year % 4)==0)
			{
				sDate.Date = (sDate.Date + 1) % 29 ;
			}
			else
			{
				sDate.Date = (sDate.Date + 1) % 28 ;
			}
		}
		else
		{
			sDate.Date = (sDate.Date + 1) % 31 ;
		}
	}

	  sAlarm.AlarmTime.Hours = hrs;
	  sAlarm.AlarmTime.Minutes = min;
	  sAlarm.AlarmTime.Seconds = 0;
	  sAlarm.AlarmTime.SubSeconds = 0;
	  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_SET;
	  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
	  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	  sAlarm.AlarmDateWeekDay = sDate.Date;
	  sAlarm.Alarm = RTC_ALARM_A;
	  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }

}

void SetSystemTime(void)
{
	LCD1602_setCursor(1,4);
	while(1)
	{
		switch(state)
		{
			case hoursInit:
				SetHours();
				break;
			case MinutesInit:
				SetMin();
				break;
			case SecondsInit:
				SetSec();
				break;
			case DayInit:
				SetDay();
				break;
			case MonthInit:
				SetMonth();
				break;
			case YearInit:
				SetYear();
				break;
			case FinishSet:
				return;
			default:
				break;
		}
		if(state <= SecondsInit)
		{
			LCD1602_setCursor(1,4);
			LCD1602_print("Set Time");
			LCD1602_setCursor(2,4);
			LCD1602_PrintInt((int)hours, (uint8_t)2);
			LCD1602_print(":");
			LCD1602_PrintInt((int)minutes, (uint8_t)2);
			LCD1602_print(":");
			LCD1602_PrintInt((int)seconds, (uint8_t)2);
		}
		else
		{
			LCD1602_setCursor(1,4);
			LCD1602_print("Set Date");
			LCD1602_setCursor(2,3);
			LCD1602_PrintInt((int)days, (uint8_t)2);
			LCD1602_print("-");
			LCD1602_PrintInt((int)months, (uint8_t)2);
			LCD1602_print("-");
			LCD1602_print("20");
			LCD1602_PrintInt((int)years, (uint8_t)2);
		}
		HAL_Delay(200);
	}
}

