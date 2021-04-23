/**
 *******************************************************************************
 * @file    CubeMon.h
 * @author  Jean
 * @brief   Liste de variables pour STM32CubeMonitor
 *******************************************************************************
 *  Created on: 11 mars 2020
 *  Modifier  : 23 avril 2021 (regrouper/renommer les data NodeRed)
 ********************************************************************************
 */

#ifndef CUBEMON_H_
#define CUBEMON_H_

/* Variables pour Mode-RED */
char nr_Prog[40];
float nr_STTS751_Temp;
float nr_HTS221_Temp;
float nr_LPS22HH_Temp;
int nr_w_Compteur;
RTC_TimeTypeDef nr_Time;
RTC_DateTypeDef nr_Date;

/* other, no need to declare here */
// char aTxBuffer[];
// uint8_t aRxBuffer[];

// TMsg msg_dat;	//for GUI dialog
// TMsg msg_cmd;	//for GUI dialog
// TMsg MsgDat;		//for GUI dialogue

#endif /* CUBEMON_H_ */
