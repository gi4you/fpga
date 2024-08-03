/*
 * console.h
 *
 *  Created on: Jun 27, 2024
 *      Author: kha
 */

#ifndef SRC_CONSOLE_H_
#define SRC_CONSOLE_H_

int   GetUserInput(char* Prompt, char* Response, int MaxChars);
void  dBpm_ConsoleCmd_Processer(char *rx_buff);

#endif /* SRC_CONSOLE_H_ */
