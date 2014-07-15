#include "parallelpiped.h"
#include "blob.h"
#include "calibration.h"
#include <cstring>
#include <cfloat>
#include <cmath>

Parallelpiped::Parallelpiped() {}
  
Parallelpiped::~Parallelpiped() {}

Parallelpiped *Parallelpiped::copy() {
    Parallelpiped *copy = new Parallelpiped();

    memcpy(copy, this, sizeof(Parallelpiped));

    return copy;
}
  
void Parallelpiped::getLimits(int limits[4][4], int nlimits[4], int varlimrel[4]) {

    int i,j;
    int limit_point_index[4];
    double limit_ext_values[4] = {DBL_MAX, - (DBL_MAX - 10.0), -(DBL_MAX - 10.0), DBL_MAX};
    double x2d,y2d;
    
    for(i=0; i<4; i++) {
        nlimits[i] = 0;
        for(j=0; j<4; j++)
            limits[i][j] = 0;
        x2d = PARALL_X2D_BASE_i(this, i);
        y2d = PARALL_Y2D_BASE_i(this, i);
      
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

        x2d = PARALL_X2D_H_i(this, i);
        y2d = PARALL_Y2D_H_i(this, i);

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

void Parallelpiped::initLimits(int limits[4][4], int nlimits[4], int varlimrel[4]) {
    int i,j;
    for(i=0;i<4;i++) {
        for(j=0;j<4;j++)
            limits[i][j]=(i==j)?1:0;
        nlimits[i]=1;
        varlimrel[i]=i;
    }
}

void Parallelpiped::get_w_l(double &w, double &l){
    double 
      X1 = PARALL_X_i(this, 1),
      Y1 = PARALL_Y_i(this, 1),
      X2 = PARALL_X_i(this, 2),
      Y2 = PARALL_Y_i(this, 2),
      X3 = PARALL_X_i(this, 3),
      Y3 = PARALL_Y_i(this, 3);
    double
      dl_x = X2 - X1,
      dl_y = Y2 - Y1,
      dw_x = X3 - X2,
      dw_y = Y3 - Y2;
    
    l = sqrt(dl_x*dl_x + dl_y*dl_y);
    w = sqrt(dw_x*dw_x + dw_y*dw_y);
}
  

void Parallelpiped::setReal3DPosition(point3D<double> *p) {
    p->x = (base_point[0].x + base_point[2].x) / 2.0;
    p->y = (base_point[0].y + base_point[2].y) / 2.0;
    p->z = 0.0;
}

//Generates a Parallelpiped considering a starting point and an orientation angle, constructing following points in inverse-clockwise direction, using the
//fact that all angles are square and using the maximal limits for the dimensions of the current expected object model.
void Parallelpiped::getFromInitial3DPoint(SceneModel *smodel, Rectangle<int> *bb2D, double x3D, double y3D, int point_index,
                                        double angle, int beta_direction, double w, double l, double h) {
    double x[4], y[4];
    double sina, cosa;
    double L = BL_INFINITY, T = BL_INFINITY, B = 0, R = 0;
    double x2d, y2d;
    int i,j;

    sina = sin(angle);
    cosa = cos(angle);

    y[point_index%4] = y3D;
    x[point_index%4] = x3D;

    for(i=0, j=(point_index+1) % 4; i < 3; i++, j++)
        switch(j%4) {
            case 0:
                x[0] = x[3] - beta_direction * l * cosa;
                y[0] = y[3] - l * sina;
                break;
            case 1:
                x[1] = x[0] + beta_direction * w * sina;
                y[1] = y[0] - w * cosa;
                break;
            case 2:
                x[2] = x[1] + beta_direction * l * cosa;
                y[2] = y[1] + l * sina;
                break;
            case 3:
                x[3] = x[2] - beta_direction * w * sina;
                y[3] = y[2] + w * cosa;
                break;
        }

    this->h = h;

    for(i = 0; i < 4; i++) {
        base_point[i].x = x[i];
        base_point[i].x = y[i];
        SceneModel::worldToImgCoords(SM_CALIB_MATRIX(smodel), x[i], y[i], 0.0, &x2d, &y2d);
        _2D_proj_0[i].x = x2d;
        _2D_proj_0[i].y = y2d;
        if(x2d < L)
            L = x2d;
        if(x2d > R)
            R = x2d;
        if(y2d < T)
            T = y2d;
        if(y2d > B)
            B = y2d;
        SceneModel::worldToImgCoords(SM_CALIB_MATRIX(smodel), x[i], y[i], h, &x2d, &y2d);
        _2D_proj_h[i].x = x2d;
        _2D_proj_h[i].y = y2d;
        if(x2d < L)
            L = x2d;
        if(x2d > R)
            R = x2d;
        if(y2d < T)
            T = y2d;
        if(y2d > B)
            B = y2d;
    }

    bb2D->initRectangle((int)L, (int)T, (int)(R - L + 1), (int)(B - T + 1));

}
