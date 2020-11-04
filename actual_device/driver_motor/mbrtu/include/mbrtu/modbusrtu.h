//
// Created by Hiep  on 10/2/2020.
//
#ifndef MODBUSPPMODBUSRTU_H
#define MODBUSPPMODBUSRTU_H
#pragma once 

#include <iostream>  /* std::cout */
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <time.h>	 /* delay */
#include <cstdint>	 /* uin8_t */
#include <stdlib.h>  


#define C3_5_time 18000
#define FN_CODE_READ        0x03
#define FN_CODE_WRITE       0x06
#define FN_CODE_DIAGNOSIS   0x08
#define FN_CODE_WRITE_MULTI 0x10

#define ADDR_ALARM_H         0x0080
#define ADDR_ALARM_L         0x0081
#define ADDR_RESET_ALARM_H   0x0180
#define ADDR_RESET_ALARM_L   0x0181
#define ADDR_SPEED0_H        0x0480
#define ADDR_SPEED0_L        0x0481
#define FEEDBACK_SPEED_H	 0x00CE
#define FEEDBACK_SPEED_L	 0x00CF
#define ADDR_WARNING_H		 0x0098
#define ADDR_WARNING_L		 0x0099
#define ADDR_TORQUE_H        0x0700
#define ADDR_TORQUE_L        0x0701
#define ADDR_TORQUE_LIMIT0_H 0x0700
#define ADDR_TORQUE_LIMIT0_L 0x0701
#define ADDR_MOTOR_CONTROL   0x007d
#define ADDR_ANALOG_MODE_L   0x10e3
#define ADDR_CONFIG_H        0x018c
#define ADDR_CONFIG_L        0x018d
#define ADDR_ACCELERATION0_H 0x0600
#define ADDR_ACCELERATION0_L 0x0601
#define ADDR_DECELERATION0_H 0x0680
#define ADDR_DECELERATION0_L 0x0681

#define ADDR_CLEAR_ALARM_RECORDS_H   0x0184
#define ADDR_CLEAR_ALARM_RECORDS_L	 0x0185
#define ADDR_CLEAR_WARNING_RECORDS_H 0x0186
#define ADDR_CLEAR_WARNING_RECORDS_L 0x0187

#define MOTOR_DIRECTOIN_STOP    0
#define MOTOR_DIRECTOIN_FORWARD 1
#define MOTOR_DIRECTOIN_REVERSE 2

#define MOTOR_STOP_BIT 	   	   0b00000000
#define MOTOR_FORWARD_BIT      0b00001000
#define MOTOR_REVERSE_BIT      0b00010000
#define MOTOR_SLOW_CHANGE_BIT  0b00100000
#define MOTOR_FREE_ON_STOP_BIT 0b10000000

#define BLVD20KM_QUERY_MAX_LEN 255
#define BLVD20KM_ERROR_CODE_INVALID_FN        0x01
#define BLVD20KM_ERROR_CODE_INVALID_ADDR      0x02
#define BLVD20KM_ERROR_CODE_INVALID_DATA      0x03
#define BLVD20KM_ERROR_CODE_SLAVE_ERROR       0x04
#define BLVD20KM_ERROR_NO_RESPONSE            0x10
#define BLVD20KM_ERROR_UNMATCH_CRC            0x11
#define BLVD20KM_ERROR_UNMATCH_ADDRESS        0x12
#define BLVD20KM_ERROR_UNMATCH_FN_CODE        0x13
#define BLVD20KM_ERROR_UNMATCH_DATA_LEN       0x14
#define BLVD20KM_ERROR_OVER_QUERY_MAX_LEN     0x15
#define BLVD20KM_ERROR_DIAGNOSIS_DATA_INVALID 0x16

#define BLVD20KM_SPEED_MIN 80
#define BLVD20KM_SPEED_MAX 4000
#define BLVD20KM_TORQUE_MAX 200

#define BLVD02KM_SPEED_MODE_USE_DIGITALS 0x0001

/* prameter serial*/
int fd; 
int rv;
fd_set set;
struct timeval timeout;
struct termios saved_tty_parameters;					/* old serial port setting (restored on close) */
struct termios Mb_tio;

/* buffer */ 
uint8_t uint8Buffer[41];
uint16_t uint16Buffer[8];
								/* new serail port setting */
void Mb_open_device(const char Mbc_port[], int Mbc_speed,	/* open device and configure it */	
					int Mbc_parity, int Mbc_bit_l,
					int Mbc_bit_s);	
void Mb_close_device();										/* close device*/	
ssize_t writeQuery(uint8_t address,uint8_t fnCode, uint8_t data[], uint16_t dataLen);
uint16_t uint8tsToUint16t(uint8_t chars[]);
uint8_t readQuery(uint8_t address, uint8_t fnCode, uint8_t data[], uint16_t dataLen); 
uint8_t writeRegister(uint8_t address,uint16_t writeAddress, uint16_t data16bit); 
uint8_t readRegisters(uint8_t address,uint16_t readStartAddress, uint16_t dataLen, uint16_t registerData[]); 
uint16_t getCRC16(uint8_t data_p[], uint16_t length, uint16_t poly);
uint16_t createMotorControl16bit(uint8_t motorDirection, bool freeLockOnStop, bool slowChange, uint8_t motorDataNum);

uint8_t writeConfigTrigger(uint8_t address);
uint8_t writeForward(uint8_t address);
uint8_t writeLock(uint8_t address);
uint8_t writeStop(uint8_t address);
uint8_t writeReverse(uint8_t address);
uint8_t writeSpeed(uint8_t address, uint16_t speed);
uint8_t writeSpeedControlMode(uint8_t address, uint16_t mode);
uint8_t writeDiagnosis(uint8_t address);
uint8_t writeResetAlarm(uint8_t address);
uint8_t clearAlarmRecords(uint8_t address);
uint8_t clearWarningRecords(uint8_t address);
uint8_t writeAcceleration(uint8_t address, uint16_t time);
uint8_t writeDeceleration(uint8_t address, uint16_t time);

uint8_t readDirection(uint8_t address,bool *forwarding, bool *reversing, bool *freeLockOnStop);
uint8_t readSpeed(uint8_t address,uint16_t *speed);
uint8_t readSpeedControlMode(uint8_t address,uint16_t *mode);
uint8_t readAlarm(uint8_t address,uint16_t *alarm);
uint8_t readWarning(uint8_t address,uint16_t *warning);
uint8_t readTorque(uint8_t address,uint16_t *torque);
uint8_t readTorqueLimit(uint8_t address,uint16_t *torque);
uint8_t readUint32t(uint8_t address,uint16_t readStartAddress, uint32_t *value);
uint8_t feedbackSpeed(uint8_t address, uint16_t *speed);
#endif //MODBUSPP_MODBUS_H