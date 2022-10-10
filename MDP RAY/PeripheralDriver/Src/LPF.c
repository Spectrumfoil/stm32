/*
 * LPF.c
 *
 *  Created on: 27 Sep 2022
 *      Author: wilso
 */

#include "stdlib.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

uint32_t Size;
uint32_t x2[1024];   // two copies of MACQ
uint32_t *Pt2;       // pointer to current
uint32_t LPFSum2;    // sum of the last Size samples
uint32_t I2;        // index to oldest

void LPF_Init2(ADC_HandleTypeDef* hadc, uint32_t size){
  uint32_t initial;

  HAL_ADC_Start(hadc);
  HAL_ADC_PollForConversion(hadc, 10);
  initial = HAL_ADC_GetValue(hadc);
  HAL_ADC_Stop(hadc);

  if(size>1024) size=1024; // max
  Size = size;
  I2 = Size - 1;
  LPFSum2 = Size*initial; // prime MACQ with initial data
  for(int i=0; i<2*Size; i++){
    x2[i] = initial;
  }
}
// calculate one filter output, called at sampling rate
// Input: new ADC data   Output: filter output, DAC data
// y(n) = (x(n)+x(n-1)+...+x(n-Size-1)/Size
uint32_t LPF_Calc2(uint32_t newdata){
  LPFSum2 = LPFSum2+newdata-x2[I2];   // subtract oldest, add newest
  x2[I2] = newdata;// save new data
  if(I2 == 0){
	  I2 = Size-1;// wrap
  }
  else{
	  I2--;// make room for data
  }
//  if(Pt2 == &x2[0]){
//    Pt2 = &x2[Size-1];// wrap
//  }
//  else{
//    Pt2--;// make room for data
//  }
//  LPFSum2 = LPFSum2+newdata -*Pt2;   // subtract oldest, add newest
//  *Pt2 = *(Pt2+Size) = newdata;     // two copies of the new data
  return LPFSum2/Size;
}
