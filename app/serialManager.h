#ifndef SERIALMANAGER_H
#define SERIALMANAGER_H

void serialMngInit(void);
void uart2write(char *str, int len);
void parseDbgMsg(char *cmd);
void uart2_printf(char *fmt, ...);
#endif
