 #ifndef _PARALLELPIPED_H_
#define _PARALLELPIPED_H_

#include <set>
#include "geometric.h"
class SceneModel;

class Parallelpiped {
  public:
    Parallelpiped();
    ~Parallelpiped();
    Parallelpiped& operator=(Parallelpiped& parall);
    Parallelpiped *copy();
    static void initLimits(int limits[4][4], int nlimits[4], int varlimrel[4]);
    void getLimits(int limits[4][4], int nlimits[4], int varlimrel[4]);
    void get_w_l(double& w, double& l);
    void setReal3DPosition(point3D<double> *p);
    void getFromInitial3DPoint(SceneModel *smodel, Rectangle<int> *bb2D, double x3D, double y3D, int point_index,
                               double angle, int beta_direction, double w, double l, double h);

    point2D<double> base_point[4]; // Four points forming the base of Parallelpiped.
    point2D<double> _2D_proj_0[4]; // 2D projections of points in the base
    point2D<double> _2D_proj_h[4]; // 2D projections of points in the base
    double h;         // Height of Parallelpiped.
};

typedef Parallelpiped *parallelpiped_t;

#define PARALL_H(_parall)	((_parall)->h)
#define PARALL_X_i(_parall, _i)	((_parall)->base_point[_i].x)
#define PARALL_Y_i(_parall, _i)	((_parall)->base_point[_i].y)
#define PARALL_POINT_i(_parall, _i)  &((_parall)->base_point[_i])

#define PARALL_X2D_BASE_i(_parall, _i)	((_parall)->_2D_proj_0[_i].x)
#define PARALL_Y2D_BASE_i(_parall, _i)	((_parall)->_2D_proj_0[_i].y)
#define PARALL_2D_BASE_POINT_i(_parall, _i)  &((_parall)->_2D_proj_0[_i])

#define PARALL_X2D_H_i(_parall, _i)	((_parall)->_2D_proj_h[_i].x)
#define PARALL_Y2D_H_i(_parall, _i)	((_parall)->_2D_proj_h[_i].y)
#define PARALL_2D_H_POINT_i(_parall, _i)  &((_parall)->_2D_proj_h[_i])

#endif /* _Parallelpiped_H_ */
