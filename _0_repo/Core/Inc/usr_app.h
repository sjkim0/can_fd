/*
 * usr_app.h
 *
 *  Created on: Dec 17, 2024
 *      Author: ksj10
 */

#ifndef INC_USR_APP_H_
#define INC_USR_APP_H_


#include "main.h"


extern uint8_t ubKeyNumber;
extern FDCAN_RxHeaderTypeDef RxHeader;
extern uint8_t RxData[8];
extern FDCAN_TxHeaderTypeDef TxHeader;
extern uint8_t TxData[8];


void apFDCANTransmit(void);


#endif /* INC_USR_APP_H_ */
