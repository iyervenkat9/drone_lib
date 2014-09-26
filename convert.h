#ifndef CONVERT_H
#define CONVERT_H
#endif

#define IEEE_MANTISSA 23
#define SLACK_BYTES 8
#define IEEE_EXPONENT 8
#define IEEE_BITS 32


unsigned char ret_exp[IEEE_EXPONENT+1];
unsigned char ret_mantissa[IEEE_MANTISSA+SLACK_BYTES+1];
unsigned char ret_Ieee[33];
char number_str[11];

		
int dec2bin(int value);

char * compute_IEEE(float x);