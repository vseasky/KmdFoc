#ifndef _HAL_FOC_TRAP_TRAJ_H
#define _HAL_FOC_TRAP_TRAJ_H

#include <hal_foc_struct.h>


void hal_TRAJ_plan(tHaltrapTraj *pTraj,float Xf, float Xi, float Vi, float Vmax, float Amax, float Dmax);

void hal_TRAJ_eval(tHaltrapTraj *pTraj);

#endif
