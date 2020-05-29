 /*
 * Proteus-II.h
 *
 *  Created on: 6-apr.-2020
 *      Author: Benjamin
 */

#ifndef SRC_PROTEUS_II_H_
#define SRC_PROTEUS_II_H_

/* Include necessary for this header file */
#include <stdint.h> /* (u)intXX_t */

/* Public prototype */
void sendCommandtoDEV(char startsignal, char command, char Length[2], char SettingIndex, char* parameter);
bool getConnectionEstablished(void);
void initiateBLE(void);
void sendTest(char* parameter);
void sendTotalBuffertoDEV(char* parameter);
void receivedConnection(char* message);
#endif /* SRC_PROTEUS_II_H_ */
