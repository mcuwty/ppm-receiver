#ifndef __PPM_H
#define __PPM_H

#include "sys.h"

#define PPM_PIN		PBout(6)

extern uint16_t ppm_channel[];
extern uint16_t ppm_channel_default[];

void PPM_Init(void);
void LogicChan2ppm(uint8_t* logic_ch);

#endif
