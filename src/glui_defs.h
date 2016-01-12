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

// forward declaration
class CSG_Node;

/// isovalue defining the isosurface
extern float isoval;

/// original/topological MC switch
extern int originalMC;

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
/// chosen implicit function
extern int curr_string;

/// cube data
extern float v[8];

/// loaded iso grid
extern FILE *isofile;

/// loaded CSG tree
extern CSG_Node *csg_root;

//-----------------------------------------------------------------------------
// main functions

/// run the MC algorithm
bool run(MarchingCubes &mc);

//-----------------------------------------------------------------------------
// drawing parameters

/// display element switch: wireframed surface
extern int wireframe;
/// display element switch: continuous surface
extern int fill;
/// display element switch: bounding cube
extern int show_cube;
/// display element switch: grid lines
extern int show_grid;

/// orthographic / perspective projection switch
extern int ortho;

/// object rotation
extern float view_rotate[16];
/// object translation
extern float obj_pos[3];

//_____________________________________________________________________________

//_____________________________________________________________________________
/// Callback ids
enum {
    LIGHT0_ENABLED_ID,
    LIGHT1_ENABLED_ID,
    LIGHT0_INTENSITY_ID,
    LIGHT1_INTENSITY_ID,

    SAVE_VIEWPORT_ID,
    LOAD_VIEWPORT_ID,

    FUN_ID,
    CASE_ID,
    CSG_ID,
    RUN_ID,

    ISO_ID,
    IPLY_ID,
    PLY_ID,
    IV_ID,
    EPS_ID,
    PPM_ID,

    RESET_ROTATION_ID,
    RESET_TRANSLATION_ID,
    RESET_ZOOM_ID,

    FLIP_ID,
    PROJ_ID,

    EXIT_ID,
};
//_____________________________________________________________________________

#endif  // _MC_GLUI_DEFS_H_
