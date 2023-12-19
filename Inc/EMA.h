/*
 * EMA.h
 *
 *  Created on: Dec 15, 2023
 *      Author: Sebastiaan
 */

#ifndef IFX_EMA_H_
#define IFX_EMA_H_

typedef struct{

	float filter_coefficient;
	float output_filter;
} IFX_EMA;

void EMA_Init(IFX_EMA *filter, float filter_coefficient);
void EMA_SetCoefficient(IFX_EMA *filter, float filter_coefficient);
float EMA_Update(IFX_EMA *filter, float input);

#endif /* INC_EMA_H_ */
