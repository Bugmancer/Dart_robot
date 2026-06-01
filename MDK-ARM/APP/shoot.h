#ifndef SHOOT_H
#define SHOOT_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "switch.h"

void shootint(const Revice_Motro_Data *shoot);
int setspeed_ctrl(int setspeed, const Revice_Motro_Data *shoot);
int shoottask(const Revice_Motro_Data *shoot, int setspeed);

#endif
