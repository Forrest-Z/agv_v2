#ifndef FX5UC_HARDWARE_H
#define FX5UC_HARDWARE_H
#pragma once 
#include <iostream>

#define ON true
#define OFF false

#define PORT0 0
#define PORT1 8
#define PORT2 16
#define PORT3 24
#define PORT4 32
#define Mbit 8192
#define SMbit 20480
/*
- Hà Nội 23/9/2020
- Thư viện này dùng để định nghĩa cấu trúc phần cứng của bộ controller PLC-FX5UC 
*/

typedef struct{
	// Phần đầu vào Input Port 0 (X0 --> X7) của FX5U 
	bool x0[7];  
	// Phần đầu vào Input Port 1 (X10 --> X17) của FX5U
	bool x1[7];  
	// Phần đầu vào Input Port 2 (X20 --> X27) của FX5U
	bool x2[7]; 
	// Phần đầu vào Input Port 3 (X30 --> X37) của FX5U
	bool x3[7]; 
	// Phần đầu vào Input Port 4 (X40 --> X47) của FX5U
	bool x4[7]; 

	// Phần đầu ra Output Port 0 (Y0 --> Y7) của FX5U 
	bool y0[7];  
	// Phần đầu ra Output Port 1 (Y10 --> Y17) của FX5U 
	bool y1[7]; 
	// Phần đầu ra Output Port 2 (Y20 --> Y27) của FX5U 
	bool y2[7]; 
	// Phần đầu ra Output Port 3 (Y30 --> Y37) của FX5U 
	bool y3[7]; 
	// Phần đầu ra Output Port 4 (Y40 --> Y47) của FX5U 
	bool y4[7]; 


	// Thanh ghi du lieu 
	uint16_t D[8000];
	uint16_t SD[10000];

}FX5U_series;


#endif