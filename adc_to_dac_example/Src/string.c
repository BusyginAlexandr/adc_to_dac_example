/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    string.c
  * @brief  String Functions.
  ******************************************************************************
  * @attention
  *
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "stm32f100xb.h"

/**
* @brief Reverse string
  */
	
void reverse(char s[])
{
  int i;
  int j;
  char c;

  for (i = 0, j = strlen(s)-1; i < j; i++, j--) 
	{
     c = s[i];
     s[i] = s[j];
     s[j] = c;
  }
}
  
/**
  * @brief This function converts a number to an ASCii string
  */

uint8_t itoa(int n, char s[])
{
  int i, sign,cnt_symb = 0;
 
  if ((sign = n) < 0) 
    n = -n;        
  i = 0;
     
	do 
	{     
    s[i++] = n % 10 + '0';   
	  cnt_symb++;
  } while ((n /= 10) > 0); 
	
  if (sign < 0)
	{
	  cnt_symb++;
    s[i++] = '-';
  }
  s[i] = '\0';
  reverse(s);
	
	return cnt_symb;
}