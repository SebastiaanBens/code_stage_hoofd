/*
 * EMA.c
 *
 *  Created on: Dec 15, 2023
 *      Author: Sebastiaan
 */

#include "EMA.h"

void EMA_Init(IFX_EMA *filter, float filter_coefficient){

	EMA_SetCoefficient(filter, filter_coefficient);
	 filter->output_filter = 0.0f;
}

void EMA_SetCoefficient(IFX_EMA *filter, float filter_coefficient){
	/* Met deze code zet je de coëfficient vast tussen 1 en 0 */
	if(filter_coefficient > 1.0f){

		filter_coefficient = 1.0f;

	} else if(filter_coefficient < 0.0f){

		filter_coefficient = 0.0f;
	}

	filter ->filter_coefficient = filter_coefficient;
}

float EMA_Update(IFX_EMA *filter, float input){

 filter ->output_filter = filter-> filter_coefficient * input + (1.0f - filter->filter_coefficient)* filter->output_filter;

 return filter->output_filter;

}




