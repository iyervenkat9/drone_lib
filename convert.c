#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "convert.h"

		
int dec2bin(int value){
	int curr_pos = IEEE_EXPONENT-1, exponent = 0;
	int compute_val = 0;

	if (value > 0xff) {
		printf("ERROR: Cannot convert number greater than 255\n");
		return 0;
	}

	memset(ret_exp, 0x00, IEEE_EXPONENT+1);

	while (curr_pos >= 0) {
		if ((compute_val + (1 << curr_pos)) > value) {
			ret_exp[IEEE_EXPONENT-curr_pos-1] = '0';
		} 
		else {
			compute_val = (1 << curr_pos) + compute_val;
			exponent = (!exponent) ? curr_pos: exponent;
			ret_exp[IEEE_EXPONENT-curr_pos-1] = '1';
		}
		curr_pos--;
	}

	return 1;
}

char * compute_IEEE(float x) {
	int curr_pos = 1, exponent = 0;
	float compute_val = 0, y = x;
	unsigned long Ieee_value = 0;
	memset(ret_mantissa, 0x00, IEEE_MANTISSA+SLACK_BYTES+1);

	if (y < 0) y = -y;


	while (curr_pos < IEEE_MANTISSA+SLACK_BYTES) {
		if ((compute_val + pow(2.0, -curr_pos)) > y) {
			ret_mantissa[curr_pos-1] = '0';
		} 
		else {
			compute_val = pow(2.0, -curr_pos) + compute_val;
			exponent = (!exponent) ? curr_pos: exponent;
			ret_mantissa[curr_pos-1] = '1';
		}
		curr_pos++;
	}

	dec2bin(127-exponent);
	memset(ret_Ieee, 0x00, 33);
	if (x > 0)
		ret_Ieee[0] = '0';
	else
		ret_Ieee[0] = '1';

	strncat(ret_Ieee, ret_exp, IEEE_EXPONENT);
	strncat(ret_Ieee + IEEE_EXPONENT + 1, ret_mantissa + exponent, IEEE_MANTISSA);
	
	curr_pos = 0;
	while (curr_pos < IEEE_BITS) {
		if (ret_Ieee[curr_pos] == '1')
			Ieee_value = Ieee_value + ((unsigned long) (1) << (IEEE_BITS - curr_pos - 1));
		curr_pos++;
	}

	memset(number_str, 0x00, 11);
	snprintf(number_str, 11, "%lu", Ieee_value);
	return number_str;
}
 
