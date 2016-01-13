/**
 * @file    glui_defs.h
 * @author  Thomas Lewiner <thomas.lewiner@polytechnique.org>
 * @author  Math Dept, PUC-Rio
 * @version 0.3
 * @date    30/05/2006
 *
 * @brief   MarchingCubes Graphical interface
 */
//________________________________________________

#ifndef _MC_GLUI_DEFS_H_
#define _MC_GLUI_DEFS_H_

#include <stdio.h>  // i/o functions
#include "MarchingCubes.h"

/// grid left extension
extern float xmin;
/// grid right extension
extern float xmax;
/// grid near extension
extern float ymin;
/// grid far extension
extern float ymax;
/// grid bottom extension
extern float zmin;
/// grid up extension
extern float zmax;

//-----------------------------------------------------------------------------
// input data

/// implicit formula
extern char formula[1024];

/// number of example implicit functions
#define NFUNS 15
/// implicit functions
extern const char *fun_list[NFUNS];
/// implicit functions
extern const char *fun_def[NFUNS];

//-----------------------------------------------------------------------------
// main functions

/// run the MC algorithm
bool run(MarchingCubes &mc, float isoval = 0.f);

#endif  // _MC_GLUI_DEFS_H_
