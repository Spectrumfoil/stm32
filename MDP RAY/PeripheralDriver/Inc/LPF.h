/*
 * LPF.h
 * @file      LPF.h
 * @brief     implements one FIR low-pass filters
 * @details   Finite length LPF<br>
 1) Size is the depth 2 to 512<br>
 2) y(n) = (sum(x(n)+x(n-1)+...+x(n-size-1))/size<br>
 3) To use a filter<br>
   a) initialize it once<br>
   b) call the filter at the sampling rate<br>
 *
 *  Created on: 27 Sep 2022
 *      Author: wilso
 */

#ifndef INC_LPF_H_
#define INC_LPF_H_

/**
 * Initialize second LPF<br>
 * Set all data to an initial value<br>
 * @param initial value to preload into MACQ
 * @param size depth of the filter, 2 to 512
 * @return none
 * @note  all three filters share the same size
 * @brief  Initialize second LPF
 */
void LPF_Init2(uint32_t initial, uint32_t size);

/**
 * Second LPF, calculate one filter output<br>
 * Called at sampling rate
 * @param newdata new ADC data
 * @return result filter output
 * @brief  FIR low pass filter
 */
uint32_t LPF_Calc2(uint32_t newdata);

#endif /* INC_LPF_H_ */
