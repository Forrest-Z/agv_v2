//
// Created by Hiep  on 10/21/2020.
//
#ifndef MLS_MESUREMENT_H
#define MLS_MESUREMENT_H
#pragma once 

#define LCP_NUM_BIT   	   0b00000111
#define MAKER_NUM_BIT 	   0b11111000
#define LINE_IS_GOOD_BIT   0b00000001
#define TRACK_LEVEL_BIT    0b00001110
#define SENSOR_FLIPPED_BIT 0b00010000
#define POLARITY_BIT 	   0b00100000
#define READING_CODE_BIT   0b01000000
#define EVENT_FLAG_BIT     0b10000000

struct lcp
{
	uint8_t lcp_nummber;
	uint8_t marker;
};  lcp = lcp;

struct status
{
	bool line_good;
	bool sensor_fipped;
	bool prolarity;
	bool reading_code
	uint8_t track_level;
};  status = status;

struct mlse
{
	uint8_t status;
	uint8_t lcp;	
};  mlse = mlse;

#endif //MLS_MESUREMENT_H