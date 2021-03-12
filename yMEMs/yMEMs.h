/*
 * yMEMs.h
 *
 * @brief Mes fonction d'accès aux MEMs de  IKS01A3
 *
 *  Created on: 30 juin 2019
 *      Author: Jean
 */

#ifndef YMEMS_H_
#define YMEMS_H_

void yMEMS_Env_Sensors_Infos(uint32_t Instance);
void yMEMS_Motion_Sensors_Infos(uint32_t Instance);

void yMEMS_Config_STTS751(void);

void yMEMS_LCD(uint32_t Instance, char row);

#endif /* YMEMS_H_ */
