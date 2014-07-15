/**
    @file        parallelepiped.c
 * @brief       parallelepiped type.
 * @author ORION team (orion@sophia.inria.fr)
 */     
 
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <cassert>
#include <float.h>

#include "common.h"
#include "parallelepiped.h"

namespace vsip {

  parallelepiped_t new_parallelepiped() {
    return (parallelepiped_t)calloc(1,sizeof(struct parallelepiped));
  }
  
  void free_parallelepiped(parallelepiped_t parall) {
    if(parall == NULL)
      return;
    free(parall);
  }
  
  parallelepiped_t copy_parallelepiped(parallelepiped_t parall)
  {

    if (!parall)
      return NULL;

    parallelepiped_t copy = new_parallelepiped();

    if(copy != NULL)
      memcpy(copy, parall, sizeof(struct parallelepiped));

    return copy;
  }
  
  void get_parall_limits(parallelepiped_t parall, int limits[4][4], int nlimits[4], int varlimrel[4]) {

    int i,j;
    int limit_point_index[4];
    double limit_ext_values[4] = {DBL_MAX, - (DBL_MAX - 10.0), -(DBL_MAX - 10.0), DBL_MAX};
    double x2d,y2d;
    
    for(i=0; i<4; i++) {
      nlimits[i] = 0;
      for(j=0; j<4; j++) 
	limits[i][j] = 0;
      x2d = PARALL_X2D_BASE_i(parall, i);
      y2d = PARALL_Y2D_BASE_i(parall, i);
      
      if(x2d < limit_ext_values[0]) { // LEFT
	limit_ext_values[0] = x2d;
	limit_point_index[0] = i;
      }
      if(y2d > limit_ext_values[1]) { // BOTTOM
	limit_ext_values[1] = y2d;
	limit_point_index[1] = i;
      }
      if(x2d > limit_ext_values[2]) { // RIGHT
	limit_ext_values[2] = x2d;
	limit_point_index[2] = i;
      }
      if(y2d < limit_ext_values[3]) { // TOP
	limit_ext_values[3] = y2d;
	limit_point_index[3] = i;
      }
      
      x2d = PARALL_X2D_H_i(parall, i);
      y2d = PARALL_Y2D_H_i(parall, i);
      
      if(x2d < limit_ext_values[0]) { // LEFT
	limit_ext_values[0] = x2d;
	limit_point_index[0] = i;
      }
      if(y2d > limit_ext_values[1]) { // BOTTOM
	limit_ext_values[1] = y2d;
	limit_point_index[1] = i;
      }
      if(x2d > limit_ext_values[2]) { // RIGHT
	limit_ext_values[2] = x2d;
	limit_point_index[2] = i;
      }
      if(y2d < limit_ext_values[3]) { // TOP
	limit_ext_values[3] = y2d;
	limit_point_index[3] = i;
      }
      
    }
    
    for(i=0; i<4; i++) {
      nlimits[limit_point_index[i]]++;
      limits[limit_point_index[i]][i] = 1;
      varlimrel[i]=limit_point_index[i];
    }
    
  }

  void init_parall_limits(int limits[4][4], int nlimits[4], int varlimrel[4]) {
    int i,j;
    for(i=0;i<4;i++) {
      for(j=0;j<4;j++) 
	limits[i][j]=(i==j)?1:0;
      nlimits[i]=1;
      varlimrel[i]=i;
    }
  }


  void get_w_l(parallelepiped_t parall, double *w, double *l){
    double 
      X1 = PARALL_X_i(parall, 1),
      Y1 = PARALL_Y_i(parall, 1),
      X2 = PARALL_X_i(parall, 2),
      Y2 = PARALL_Y_i(parall, 2),
      X3 = PARALL_X_i(parall, 3),
      Y3 = PARALL_Y_i(parall, 3);
    double
      dl_x = X2 - X1,
      dl_y = Y2 - Y1,
      dw_x = X3 - X2,
      dw_y = Y3 - Y2;
    
    *l = sqrt(dl_x*dl_x + dl_y*dl_y);
    *w = sqrt(dw_x*dw_x + dw_y*dw_y);
  }
  
} // end namespace vsip

