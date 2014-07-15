#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <math.h>
#include <iostream>

#include "world_def.h"
#include "geometric.h"
#include "intervals.h"
#include "blob.h"
#include "DRAW/drawInterface.h"
namespace world {

    AOI::AOI() {
        QSharedPointer < polygon3D<double> > p1(new polygon3D<double>());
        poly3DOnGround = p1;
        QSharedPointer < polygon2D<double> > p2(new polygon2D<double>());
        poly2DOnGround = p2;
        QSharedPointer < polygon2D<double> > p3(new polygon2D<double>());
        outlineOnImg = p3;
    }

    AOI::~AOI() { }

    //Test if (x,y) is inside a ROI
    bool AOI::pointInside(double x, double y) {
        return poly2DOnGround->pointInPolygon(x, y, true);
    }

    //Tests if blob is inside aoi ground on 3D
    bool AOI::blobInsideGround3D(Blob *pBlob) {
        bool val = false;
  
        // Projection 3d to 2d
        QSharedPointer<polygon2D<double> > pPolygon2d = poly3DOnGround->makePolygon2D();
	
        // Test if blob center is inside aoi ground...
        if (pPolygon2d->pointInPolygon(BLOB_3D_X(pBlob), BLOB_3D_Y(pBlob), 0))
            val = true;

        return val;
    }

    Zone::Zone() {
        QSharedPointer < polygon3D<double> > p1(new polygon3D<double>());
        outline3D = p1;
        QSharedPointer < polygon2D<int> > p2(new polygon2D<int>());
        outline = p2;
    }

    Zone::~Zone() {}


    Zone2D::Zone2D() {
        type = Z2D_polygon;
    }

    Zone2D::~Zone2D() {}


    Zone2DPolygon::Zone2DPolygon(): Zone2D() {
        QSharedPointer < polygon2D<int> > p(new polygon2D<int>());
        outline = p;
    }

    Zone2DPolygon::~Zone2DPolygon() {}

    bool Zone2DPolygon::setParameters(QDomNode &params) {
        while( !params.isNull() ) {
            QDomElement f = params.toElement();
            if( !f.isNull() ) {
                if( f.tagName() == "OutlineList" ) {
                    QSharedPointer< polygon2D<int> > pol(new polygon2D<int>());
                    outline = pol;

                    QDomNode p = params.firstChild();
                    while( !p.isNull() ) {
                        QDomElement g = p.toElement();
                        if( !g.isNull() ) {
                            if( g.tagName() == "Point" ) {
                                point2D<int> point;
                                point.x = g.attribute("X","0").toInt();
                                point.y = g.attribute("Y","0").toInt();
                                pol->points.push_back(point);
                            }
                        }
                        p = p.nextSibling();
                    }
                }
            }
            params = params.nextSibling();
        }
        return true;
    }

    bool Zone2DPolygon::draw(QPainter &p, QColor color) {
        drawInterface::showImagePolygon(p, ZONE_OUTLINE(this),color);
    }



    Zone2DEllipse::Zone2DEllipse(): Zone2D() {
    }

    Zone2DEllipse::~Zone2DEllipse() {
    }


    bool Zone2DEllipse::setParameters(QDomNode &params) {
        while( !params.isNull() ) {
            QDomElement f = params.toElement();
            if( !f.isNull() ) {
                if( f.tagName() == "Center" ) {
                    x = f.attribute("X","0").toInt();
                    y = f.attribute("Y","0").toInt();
                } else if( f.tagName() == "Radius" ) {
                    rx = f.attribute("RX","0").toInt();
                    ry = f.attribute("RY","0").toInt();
                }
            }
            params = params.nextSibling();
        }
        return true;

    }

    bool Zone2DEllipse::draw(QPainter &p, QColor color) {
        QPen pen;
        pen.setWidth(0);
        p.setPen(pen);
        p.drawEllipse(QPointF(x,y),rx,ry);
        return true;
    }


    ZoneH::ZoneH() {
    }

    ZoneH::~ZoneH() {}

    ZoneHPolygon::ZoneHPolygon() {
        QSharedPointer < polygon2D<double> > p1(new polygon2D<double>());
        outline2D = p1;
        QSharedPointer < polygon2D<int> > p2(new polygon2D<int>());
        outline = p2;
    }

    ZoneHPolygon::~ZoneHPolygon() {}

    bool ZoneHPolygon::setParameters(QDomNode &params, homography_matrix h_matrix) {
        while( !params.isNull() ) {
            QDomElement f = params.toElement();
            if( !f.isNull() ) {
                if( f.tagName() == "OutlineList" ) {
                    QSharedPointer< polygon2D<double> > pol(new polygon2D<double>());
                    outline2D = pol;

                    QDomNode p = params.firstChild();
                    while( !p.isNull() ) {
                        QDomElement g = p.toElement();
                        if( !g.isNull() ) {
                            if( g.tagName() == "Point" ) {
                                point2D<double> point;
                                point.x = g.attribute("X","0").toDouble();
                                point.y = g.attribute("Y","0").toDouble();
                                pol->points.push_back(point);
                            }
                        }
                        p = p.nextSibling();
                    }
                    outline = pol->projectImagePolygon2D(h_matrix);
                }
            }
            params = params.nextSibling();
        }
        return true;

    }

    bool ZoneHPolygon::draw(QPainter &p, QColor &color, QTransform &h_trans) {
        drawInterface::showImagePolygon(p, ZONE_OUTLINE(this),color);
    }

    double ZoneHPolygon::distanceToCenter(double x, double y) {
        return 0;
    }


    ZoneHEllipse::ZoneHEllipse(): ZoneH() {
    }

    ZoneHEllipse::~ZoneHEllipse() {
    }

    double ZoneHEllipse::distanceToCenter(double x, double y) {
        double dx = x - this->x, dy = y - this->y;
        return sqrt(dx*dx + dy*dy);
    }


    bool ZoneHEllipse::setParameters(QDomNode &params, homography_matrix h_matrix) {
        while( !params.isNull() ) {
            QDomElement f = params.toElement();
            if( !f.isNull() ) {
                if( f.tagName() == "Center" ) {
                    x = f.attribute("X","0").toDouble();
                    y = f.attribute("Y","0").toDouble();
                } else if( f.tagName() == "Radius" ) {
                    rx = f.attribute("RX","0").toDouble();
                    ry = f.attribute("RY","0").toDouble();
                }
            }
            params = params.nextSibling();
        }

        return true;

    }

    bool ZoneHEllipse::draw(QPainter &p, QColor &color, QTransform &h_trans) {
        double x2,y2;
        p.setTransform(h_trans);
        h_trans.map(10,10,&x2,&y2);
        //std::cout << "1:" << x2 << "," << y2 << std::endl;
        h_trans.map(20,20,&x2,&y2);
        //std::cout << "2:" << x2 << "," << y2 << std::endl;
        h_trans.map(40,40,&x2,&y2);
        //std::cout << "3:" << x2 << "," << y2 << std::endl;
        h_trans.map(60,60,&x2,&y2);
        //std::cout << "4:" << x2 << "," << y2 << std::endl;

        //p.setRenderHint(QPainter::Antialiasing,true);
        QPen pen;
        pen.setWidth(0);
        pen.setColor(color);
        p.setPen(pen);
        QPainterPath path;
        path.addEllipse(x - rx, y - ry, 2*rx, 2*ry);
        p.drawPath(path);
        //p.drawEllipse(x - rx, y - ry, 2*rx, 2*ry);
        return true;
    }


    Area::Area() {}

    Area::~Area() {}

    Wall::Wall() {

        QSharedPointer< point3D<double> > spoint(new point3D<double>());
        startPoint = spoint;
        QSharedPointer< point3D<double> > epoint(new point3D<double>());
        endPoint = epoint;
    }

    Wall::~Wall() {}

    Wall2D::Wall2D() {
        QSharedPointer < polygon2D<double> > o(new polygon2D<double>());
        outlineOnImage = o;
    }

    Wall2D::~Wall2D() {}

    ContextObject::ContextObject() {
        QSharedPointer < polygon3D<double> > o(new polygon3D<double>());
        outline = o;
        QSharedPointer < polygon2D<double> > p1(new polygon2D<double>());
        outline2DOnImage = p1;
        QSharedPointer < polygon2D<double> > p2(new polygon2D<double>());
        poly2DOfObjectBase = p2;
        QSharedPointer < polygon2D<double> > p3(new polygon2D<double>());
        poly2DOfObjectRoof = p3;
        QSharedPointer < polygon2D<double> > p4(new polygon2D<double>());
        poly2DOfOutline = p4;
    }

    ContextObject::~ContextObject() {}

    Camera::Camera() {
        QSharedPointer < point3D<double> > p1(new point3D<double>());
        location = p1;
        QSharedPointer < point3D<double> > p2(new point3D<double>());
        focalPoint = p2;
        QSharedPointer < point2D<double> > p3(new point2D<double>());
        focalPoint2D = p3;
    }

    Camera::~Camera() { }

bool Zone::rectangleInside(Rectangle<int> *rect) {
    return false;
}


bool Zone::pointInZone(double x, double y) {
    return outline->pointInPolygon(x, y, false);
}



bool Zone2DPolygon::rectangleInside(Rectangle<int> *rect) {

}

bool Zone2DPolygon::pointInZone(double x, double y) {
    return outline->pointInPolygon(x, y, false);
}

bool Zone2DEllipse::rectangleInside(Rectangle<int> *rect) {

}

bool Zone2DEllipse::pointInZone(double x, double y) {
    double dx = (this->x - x)/rx, dy = (this->y - y)/ry;
    return dx*dx + dy*dy < 1.0 ? true : false;
}


bool ZoneHPolygon::rectangleInside(Rectangle<int> *rect) {

}

bool ZoneHPolygon::pointInZone(double x, double y) {
    return outline2D->pointInPolygon(x, y, false);
}

bool ZoneHEllipse::rectangleInside(Rectangle<int> *rect) {

}

bool ZoneHEllipse::pointInZone(double x, double y) {
    double dx = (this->x - x)/rx, dy = (this->y - y)/ry;
    return dx*dx + dy*dy < 1.0 ? true : false;
}


/*



int rectangle_in_zone(rect_t rect, wdef_zone_t zone)
{
    polygon2D<double> *rpoly;
    int res;

    // First make a polygon from the rectangle
    rpoly = make_rectangle_polygon(rect);
    POLYGON_FLAG_MAX(rpoly) = 0;
    res = polygon_in_polygon(rpoly, ZONE_OUTLINE(zone));
    destroy_fpolygon2d(rpoly);
    return res;
}

// test if (x,y) is inside a area
int pointxy_in_area(int x, int y, wdef_area_t area)
{
    int i;
 
    for(i=0; i< AREA_N_PARTS(area); i++)
    {
        //if(_bb_pointxy_in_polygon(x, y, ZONE_OUTLINE(AREA_NTH_PART(area, i)), 0))
	  if(pointxy_in_polygon(x, y, ZONE_OUTLINE(AREA_NTH_PART(area, i)), 0))
        return 1;
    }
    return 0;
}                         

bool zone_in_area(std::string i_name, wdef_area_t area) {
  int i;
  for(i=0; i< AREA_N_PARTS(area); i++) {
    if(std::string(ZONE_NAME(AREA_NTH_PART(area, i)))==i_name)
      return true;
  }
  return false;
}


// Add Mag : 24/05/02
int get_base_center_object(polygon3D<double> *pPolygon3d,
			   fpoint_2d_t pCenter)
{
  // Declaration
  int i;
  int NbPoints;
  struct fpoint_2d *pPt;
  polygon2D<double> *pPolygon2d;

  // Init 
  POINT_2D_X(pCenter) = 0.0;
  POINT_2D_Y(pCenter) = 0.0;

  // Get polygon on ground
  pPolygon2d = make_poly2d_from_poly3d(pPolygon3d);
 
  // Get NbPoint
  NbPoints = pPolygon2d->n_points;

  // Get points
  pPt = pPolygon2d->pts_array;  

  // Fill point
  for(i=0; i<NbPoints; i++) {
	POINT_2D_X(pCenter) += POINT_2D_X(&pPt[i]);
	POINT_2D_Y(pCenter) += POINT_2D_Y(&pPt[i]);
  }

  // Division
  POINT_2D_X(pCenter) /= NbPoints;
  POINT_2D_Y(pCenter) /= NbPoints;

  // Delete
  destroy_fpolygon2d(pPolygon2d);

  return 1;
}

int get_3d_coor_top(int x, int y, int size_2d,
		    double *x_ptr, double *y_ptr, double *z_ptr)
{
  return 1;
}    

int change_coordsys(double x, double y, double z,
					coordsys_t cd_sys,
					double *res_x, double *res_y, double *res_z)
{
    point_3d_t org = COORDSYS_ORIGIN(cd_sys);
    vector_3d_t v0, v1, v2;

    v0 = COORDSYS_AXIS(cd_sys, 0);
    v1 = COORDSYS_AXIS(cd_sys, 1);
    v2 = COORDSYS_AXIS(cd_sys, 2);

    if (res_x != NULL)
	*res_x = (VECTOR_3D_X(v0) * x + VECTOR_3D_X(v1) * y +
		  VECTOR_3D_X(v2) * z + POINT_3D_X(org));
    if (res_y != NULL)
	*res_y = (VECTOR_3D_Y(v0) * x + VECTOR_3D_Y(v1) * y +
		  VECTOR_3D_Y(v2) * z + POINT_3D_Y(org));
    if (res_z != NULL)
	*res_z = (VECTOR_3D_Z(v0) * x + VECTOR_3D_Z(v1) * y +
		  VECTOR_3D_Z(v2) * z + POINT_3D_Z(org));
    return 0;
}


//Set all wall parameters
wdef_2Dwall_t new_2Dwall(wdef_wall_t wall, struct scene_model *sm_node) {

  int i;
  double A[2], B[2], C[2], K[6];
  double x[2], y[2];
  double fX2D, fY2D, xin, yin;
  double pp[12];
  struct fpoint_2d p1, p2, p3, p4;
  double a, b;
  interval int1, int2, result;
  int is_function;
  blob_occl_type gradient_pertinence = MM_OCCL_NONE;
  double PIXEL_ERROR = 2.0;

  new_interval(&int1);
  new_interval(&int2);
  new_interval(&result);

  wdef_2Dwall_t wall2d = (wdef_2Dwall_t)calloc(1,sizeof(struct wdef_2Dwall));

  if(!wall || !sm_node)
    return NULL;
  
  WALL2D_IDENT(wall2d) = WALL_IDENT(wall);
  WALL2D_NAME(wall2d) = WALL_NAME(wall);
  WALL2D_MAXH(wall2d) = WALL_HEIGHT(wall);
  WALL2D_NEXT(wall2d) = NULL;

  x[0] = WALL_START_POINT_X(wall);
  y[0] = WALL_START_POINT_Y(wall);
  x[1] = WALL_END_POINT_X(wall);
  y[1] = WALL_END_POINT_Y(wall);

  for(i = 0;i < 12; i++)
    pp[i]  = SM_CALIB_MATRIX_NTH_ELEM(sm_node, i);

  //Set 2d limit points parameters
  for(i = 0;i < 2; i++) {
    A[i] = WALL2D_PARAM_A_i(wall2d, i) = pp[0]*x[i] + pp[1]*y[i] + pp[3];
    B[i] = WALL2D_PARAM_B_i(wall2d, i) = pp[4]*x[i] + pp[5]*y[i] + pp[7];
    C[i] = WALL2D_PARAM_C_i(wall2d, i) = pp[8]*x[i] + pp[9]*y[i] + pp[11];
  }

  //Set 2d line parameters
  K[0] = WALL2D_PARAM_K_i(wall2d,0) = (B[0] - B[1])*pp[10] - (C[0] - C[1])*pp[6];
  K[1] = WALL2D_PARAM_K_i(wall2d,1) = B[0]*C[1] - B[1]*C[0];
  K[2] = WALL2D_PARAM_K_i(wall2d,2) = (A[0] - A[1])*pp[6]  - (B[0] - B[1])*pp[2];
  K[3] = WALL2D_PARAM_K_i(wall2d,3) = A[0]*B[1] - A[1]*B[0];
  K[4] = WALL2D_PARAM_K_i(wall2d,4) = (A[0] - A[1])*pp[10] - (C[0] - C[1])*pp[2];
  K[5] = WALL2D_PARAM_K_i(wall2d,5) = A[0]*C[1] - A[1]*C[0];
  
  //Get line and limit points for lowest line 
  is_function = get_2Dwall_line(wall2d, 0.0, &a, &b);
  world_to_img_coords(SM_CALIB_MATRIX(sm_node), x[0], y[0], 0.0, &POINT_2D_X(&p1), &POINT_2D_Y(&p1));
  world_to_img_coords(SM_CALIB_MATRIX(sm_node), x[1], y[1], 0.0, &POINT_2D_X(&p2), &POINT_2D_Y(&p2));

  world_to_img_coords(SM_CALIB_MATRIX(sm_node), x[0], y[0], WALL_HEIGHT(wall), &POINT_2D_X(&p3), &POINT_2D_Y(&p3));
  world_to_img_coords(SM_CALIB_MATRIX(sm_node), x[1], y[1], WALL_HEIGHT(wall), &POINT_2D_X(&p4), &POINT_2D_Y(&p4));

  //Set visibility
  WALL2D_VISIBLE(wall2d) = 0; //Start assuming not visible
  if(is_function) {

    new_interval(&int1, 0.0, SM_IMG_WIDTH(sm_node) - 1);
    new_interval(&int2, POINT_2D_X(&p1), POINT_2D_X(&p2));
    if( intersect(&result, &int1, &int2) ) { //Check possibly visible X2D interval

      new_interval(&int1, 0.0, SM_IMG_HEIGHT(sm_node));
      new_interval(&int2, a*INTERVAL_X1(&result) + b, a*INTERVAL_X2(&result) + b);
      if( intersect(&result, &int1, &int2) ) //Check Y2D on X2D possibly visible interval
	WALL2D_VISIBLE(wall2d) = 1;
    }
  } else {
    if( POINT_2D_X(&p1) < SM_IMG_HEIGHT(sm_node) && POINT_2D_X(&p1) > SM_IMG_HEIGHT(sm_node)) {
      new_interval(&int1, 0.0, SM_IMG_HEIGHT(sm_node) - 1); 
      new_interval(&int2, POINT_2D_Y(&p1),POINT_2D_Y(&p2));
      if( intersect(&result, &int1, &int2) )
	WALL2D_VISIBLE(wall2d) = 1;
    }
  }

  //Get focal point projected on floor and transformed to image coordinates
  fX2D = SM_CAMERA_X2D_FOC_POINT(sm_node);
  fY2D = SM_CAMERA_Y2D_FOC_POINT(sm_node);

  //Set pertinence on limits

  WALL2D_OCC_PERT(wall2d) = MM_OCCL_NONE;  

  if(is_function) {
    xin = (fX2D + a*(fY2D - b))/(a*a + 1);
    yin = a*xin + b;
  } else {
    xin = POINT_2D_X(&p1);
    yin = fY2D;
  }

  if(xin + PIXEL_ERROR < fX2D)
    gradient_pertinence = (blob_occl_type)((int)gradient_pertinence | MM_CAM_OCCL_LEFT);
  else if(xin - PIXEL_ERROR > fX2D)
    gradient_pertinence = (blob_occl_type)((int)gradient_pertinence | MM_CAM_OCCL_RIGHT);

  if(yin + PIXEL_ERROR < fY2D)
    gradient_pertinence = (blob_occl_type)((int)gradient_pertinence | MM_CAM_OCCL_TOP);
  else if(yin - PIXEL_ERROR > fY2D)
    gradient_pertinence = (blob_occl_type)((int)gradient_pertinence | MM_CAM_OCCL_BOTTOM);
    

  if(POINT_2D_X(&p1) <= fX2D || POINT_2D_X(&p2) <= fX2D)
    WALL2D_OCC_PERT(wall2d) = (blob_occl_type)(((int)WALL2D_OCC_PERT(wall2d) | MM_CAM_OCCL_LEFT) & (int)gradient_pertinence);

  if(POINT_2D_X(&p1) >= fX2D || POINT_2D_X(&p2) >= fX2D)
    WALL2D_OCC_PERT(wall2d) = (blob_occl_type)(((int)WALL2D_OCC_PERT(wall2d) | MM_CAM_OCCL_RIGHT) & (int)gradient_pertinence);

  if(POINT_2D_Y(&p1) <= fY2D || POINT_2D_Y(&p2) <= fY2D)
    WALL2D_OCC_PERT(wall2d) = (blob_occl_type)(((int)WALL2D_OCC_PERT(wall2d) | MM_CAM_OCCL_TOP) & (int)gradient_pertinence);

  if(POINT_2D_Y(&p1) >= fY2D || POINT_2D_Y(&p2) >= fY2D)
    WALL2D_OCC_PERT(wall2d) = (blob_occl_type)(((int)WALL2D_OCC_PERT(wall2d) | MM_CAM_OCCL_BOTTOM) & (int)gradient_pertinence);
  if(is_function)
    WALL2D_DIS_TYPE(wall2d) = (fY2D <= fX2D*a + b) ? 0 : 1;
  else
    WALL2D_DIS_TYPE(wall2d) = (fX2D <= POINT_2D_X(&p2)) ? 0 : 1;
  
  //Set outline 2D of a wall
  WALL2D_OUTLINE_ON_IMAGE(wall2d) = make_fpolygon2d();
  fpolygon_2d_add_pointxy(WALL2D_OUTLINE_ON_IMAGE(wall2d), POINT_2D_X(&p1), POINT_2D_Y(&p1));
  fpolygon_2d_add_pointxy(WALL2D_OUTLINE_ON_IMAGE(wall2d), POINT_2D_X(&p3), POINT_2D_Y(&p3));
  fpolygon_2d_add_pointxy(WALL2D_OUTLINE_ON_IMAGE(wall2d), POINT_2D_X(&p4), POINT_2D_Y(&p4));
  fpolygon_2d_add_pointxy(WALL2D_OUTLINE_ON_IMAGE(wall2d), POINT_2D_X(&p2), POINT_2D_Y(&p2));
  compute_bounding_rectangle(WALL2D_OUTLINE_ON_IMAGE(wall2d), &POLYGON_2D_BB(WALL2D_OUTLINE_ON_IMAGE(wall2d)));

  //Set wall line segment definitions
  set_wall_segment(WALL2D_SEGMENT(wall2d,0), POINT_2D_X(&p1), POINT_2D_Y(&p1), POINT_2D_X(&p3), POINT_2D_Y(&p3), 2);
  set_wall_segment(WALL2D_SEGMENT(wall2d,1), POINT_2D_X(&p3), POINT_2D_Y(&p3), POINT_2D_X(&p4), POINT_2D_Y(&p4), 1);
  set_wall_segment(WALL2D_SEGMENT(wall2d,2), POINT_2D_X(&p4), POINT_2D_Y(&p4), POINT_2D_X(&p2), POINT_2D_Y(&p2), 2);
  set_wall_segment(WALL2D_SEGMENT(wall2d,3), POINT_2D_X(&p2), POINT_2D_Y(&p2), POINT_2D_X(&p1), POINT_2D_Y(&p1), 0);

  //Set 3D segment of the base
  set_wall_segment(WALL2D_SEGMENT_BASE3D(wall2d), x[0], y[0], x[1], y[1], 0);

  return wall2d;
}
*/

    //Returns the parameters a and b for a image plane line y2d = a*x2d + b, given wall and h
    bool Wall2D::get2DWallLine(double h, double *a, double *b) {

        double den = (K[4]*h + K[5]);

        if(den != 0.0) {
            if(a != NULL)
                *a = (K[0]*h + K[1])/den;
            if(b != NULL)
                *b = (K[2]*h + K[3])/den;
            return true;
        }

        if(a != NULL)
            *a = 0.0;
        if(b != NULL)
            *b = 0.0;
        return false;
    }

    //Returns the limit 2D image points p1 and p2, given wall and h
    void Wall2D::get2DwallLimitPoints(double h, perspective_matrix persp_mat, point2D<double>* p1, point2D<double>* p2) {
        double pp2  = persp_mat[2],
               pp6  = persp_mat[6],
               pp10 = persp_mat[10];

        if(p1 != NULL) {
            POINT_2D_X(p1) = (pp2*h + A[0])/(pp10*h + C[0]);
            POINT_2D_Y(p1) = (pp6*h + B[0])/(pp10*h + C[0]);
        }
        if(p2 != NULL) {
            POINT_2D_X(p2) = (pp2*h + A[1])/(pp10*h + C[1]);
            POINT_2D_Y(p2) = (pp6*h + B[1])/(pp10*h + C[1]);
        }
    }

    /*
//Returns 1 if point (x2d,y2d) is inside of pertinent zone according to wall and height h
int point_is_pertinent(wdef_2Dwall_t wall, double h, persp_matrix persp_mat, double x2d, double y2d) {
  double a, b;
  struct fpoint_2d p;

  if(!get_2Dwall_line(wall, h, &a, &b)) { //if x is constant
    get_2Dwall_limit_points(wall, h, persp_mat, &p, NULL);

    if(WALL2D_DIS_TYPE(wall))
      return (x2d > POINT_2D_X(&p)) ? 1 : 0;
    
    return (x2d <= POINT_2D_X(&p)) ? 1 : 0;
  }

  //if wall is function

  if(WALL2D_DIS_TYPE(wall))
    return (y2d > x2d*a + b) ? 1 : 0;

  return (y2d <= x2d*a + b) ? 1 : 0;

}

//Returns 1 if point p is inside of pertinent zone according to wall, for a point in the floor (optimized for floor points)
int point_in_floor_is_pertinent(wdef_2Dwall_t wall, persp_matrix persp_mat, double x2d, double y2d) {
  double *K = WALL2D_PARAM_K(wall);
  double den = K[5];
  struct fpoint_2d p;

  if(K[5] == 0) { //if x is constant
    get_2Dwall_limit_points(wall, 0.0, persp_mat, &p, NULL);

    if(WALL2D_DIS_TYPE(wall))
      return (x2d > POINT_2D_X(&p)) ? 1 : 0;

    return (x2d <= POINT_2D_X(&p)) ? 1 : 0;
  }

  //if wall is function
  double
    a = K[1]/den,
    b = K[3]/den;

  if(WALL2D_DIS_TYPE(wall))
    return (y2d > x2d*a + b) ? 1 : 0;

  return (y2d <= x2d*a + b) ? 1 : 0;

}

int set_object_outline_from_wall_list(struct polygon_3d *outline, struct wdef_wall *wall_list) {
  struct wdef_wall *current_wall = wall_list;
  int walls_number = 0;
  int *filled_in;
  int i, wall_ref;
  if(outline == NULL) {
    fprintf(stderr, "set_object_outline_from_wall_list: no valid target outline...\n");
    return -4;
  }

  if(wall_list == NULL) {
    fprintf(stderr, "set_object_outline_from_wall_list: no valid wall_list...\n");
    return -4;
  }
  
  while(current_wall != NULL) {
    walls_number++;
    current_wall = WALL_NEXT(current_wall);
  }

  outline->n_points  = walls_number;
  outline->pts_array = (struct point_3d *)calloc(walls_number,sizeof(struct point_3d ));
  
  filled_in = (int *)calloc(walls_number, sizeof(int));

  current_wall = wall_list;
  POINT_3D_X(&(outline->pts_array[0])) = WALL_START_POINT_X(current_wall);
  POINT_3D_Y(&(outline->pts_array[0])) = WALL_START_POINT_Y(current_wall);
  POINT_3D_Z(&(outline->pts_array[0])) = WALL_START_POINT_Z(current_wall);
  POINT_3D_X(&(outline->pts_array[1])) = WALL_END_POINT_X(current_wall);
  POINT_3D_Y(&(outline->pts_array[1])) = WALL_END_POINT_Y(current_wall);
  POINT_3D_Z(&(outline->pts_array[1])) = WALL_END_POINT_Z(current_wall);
  wall_ref = 0;
  filled_in[0] = wall_ref;
  filled_in[1] = wall_ref;

  current_wall = WALL_NEXT(current_wall);
  wall_ref++;
  for(i = 2;i < walls_number; i++) {
    if( current_wall != NULL && equal_points3D( &(outline->pts_array[i-1]), &(WALL_START_POINT(current_wall) ) ) ) { //case where walls are ordered
      POINT_3D_X(&(outline->pts_array[i])) = WALL_END_POINT_X(current_wall);
      POINT_3D_Y(&(outline->pts_array[i])) = WALL_END_POINT_Y(current_wall);
      POINT_3D_Z(&(outline->pts_array[i])) = WALL_END_POINT_Z(current_wall);
      filled_in[i] = wall_ref;
    } else if( current_wall != NULL && equal_points3D( &(outline->pts_array[i-1]), &(WALL_END_POINT(current_wall) ) ) ) { //case where walls are ordered
      POINT_3D_X(&(outline->pts_array[i])) = WALL_START_POINT_X(current_wall);
      POINT_3D_Y(&(outline->pts_array[i])) = WALL_START_POINT_Y(current_wall);
      POINT_3D_Z(&(outline->pts_array[i])) = WALL_START_POINT_Z(current_wall);
      filled_in[i] = wall_ref;
      current_wall = WALL_NEXT(current_wall);
    } else { //if walls are not ordered
      current_wall = WALL_NEXT(wall_list);
      wall_ref = 1;
      while( current_wall != NULL ) {
	if( equal_points3D( &(outline->pts_array[i-1]), &(WALL_START_POINT(current_wall) ) ) ) { //case where walls are ordered
	  POINT_3D_X(&(outline->pts_array[i])) = WALL_END_POINT_X(current_wall);
	  POINT_3D_Y(&(outline->pts_array[i])) = WALL_END_POINT_Y(current_wall);
	  POINT_3D_Z(&(outline->pts_array[i])) = WALL_END_POINT_Z(current_wall);
	  filled_in[i] = wall_ref;
	  break;
	} else if( current_wall != NULL && equal_points3D( &(outline->pts_array[i-1]), &(WALL_END_POINT(current_wall) ) ) ) { //case where walls are ordered
	  POINT_3D_X(&(outline->pts_array[i])) = WALL_START_POINT_X(current_wall);
	  POINT_3D_Y(&(outline->pts_array[i])) = WALL_START_POINT_Y(current_wall);
	  POINT_3D_Z(&(outline->pts_array[i])) = WALL_START_POINT_Z(current_wall);
	  filled_in[i] = wall_ref;
	  break;
	}
	wall_ref++;	
	current_wall = WALL_NEXT(current_wall);
      }

      if (current_wall == NULL) {
	fprintf(stderr, "set_object_outline_from_wall_list: No continuity in walls definition. Outline cannot be constructed to form a polygon.\n");
	return -4;
      }


    }
    if (current_wall == NULL || WALL_NEXT(current_wall) == NULL) {
      current_wall = WALL_NEXT(wall_list);
      wall_ref = 1;
    } else {
      current_wall = WALL_NEXT(current_wall);
      wall_ref++;
    }

  }

  free(filled_in);
  return 0;
}

int equal_points3D(struct point_3d *p1, struct point_3d *p2) {
  return (    POINT_3D_X(p1) == POINT_3D_X(p2)
	   && POINT_3D_Y(p1) == POINT_3D_Y(p2) 
	   && POINT_3D_Z(p1) == POINT_3D_Z(p2) 
	 ) ? 1 : 0;
}

struct wdef_wall *set_object_wall_list_from_outline(struct polygon_3d *outline, double height) {
  struct wdef_wall * prev = NULL;
  struct wdef_wall * cur;
  struct wdef_wall * res = NULL;
  char name_buff[20];
  int i, dim;

  if(outline == NULL) {
    fprintf(stderr, "set_object_wall_list_from_outline: Error: no valid source outline...\n");
    return res;
  }

  if ( (dim = outline->n_points) == 0) {
    fprintf(stderr, "set_object_wall_list_from_outline: Error: empty outline...\n");
    return res;
  }

  res = (struct wdef_wall *)calloc(dim, sizeof(struct wdef_wall));
  
  for(i = 0; i < dim; i++ , prev = cur) {
    cur = & res[i];

    WALL_IDENT(cur) = i;
    sprintf(name_buff, "Wall %d", i);
    WALL_NAME(cur) = (char *)calloc(strlen(name_buff)+1,sizeof(char));
    sprintf(WALL_NAME(cur), "%s", name_buff);
    WALL_PROPERTIES(cur) = (wdef_prop_t) calloc(1, sizeof(union wdef_prop));
    WALL_IS_TRANSPARENT(cur) = 0;
    WALL_IS_VIRTUAL(cur)     = 0;
    WALL_START_POINT_X(cur)  = POINT_3D_X(&(outline->pts_array[i]));
    WALL_START_POINT_Y(cur)  = POINT_3D_Y(&(outline->pts_array[i]));
    WALL_START_POINT_Z(cur)  = POINT_3D_Z(&(outline->pts_array[i]));
    WALL_END_POINT_X(cur)    = POINT_3D_X(&(outline->pts_array[(i + 1)%dim]));
    WALL_END_POINT_Y(cur)    = POINT_3D_Y(&(outline->pts_array[(i + 1)%dim]));
    WALL_END_POINT_Z(cur)    = POINT_3D_Z(&(outline->pts_array[(i + 1)%dim]));
    WALL_HEIGHT(cur)         = height;
    
    if(prev)
      prev -> next = cur;

  }

  WALL_NEXT(cur) = NULL;

  return res;

}

void set_wall_as_line_equation(wdef_wall_t wall) {
  double dx = WALL_END_POINT_X(wall) - WALL_START_POINT_X(wall); 
  double dy = WALL_END_POINT_Y(wall) - WALL_START_POINT_Y(wall); 

  if(dx == 0.0) {
    WALL_SEGMENT_IS_FUNCTION(wall) = 0;
    WALL_B(wall) = WALL_END_POINT_X(wall);
    return;
  }

  WALL_SEGMENT_IS_FUNCTION(wall) = 1;
  WALL_A(wall) = dy/dx;
  WALL_B(wall) = WALL_END_POINT_Y(wall) - WALL_A(wall)*WALL_END_POINT_X(wall);
    
}
*/
    void WallSegment::setWallSegment(double i_x1, double i_y1, double i_x2, double i_y2, int i_in_height) {
  
        x1 = i_x1;
        y1 = i_y1;
        x2 = i_x2;
        y2 = i_y2;
        inHeight = i_in_height;
        if(x2 - x1 == 0.0) {
            vertical = true;
            slope = 0;
            intercept = 0;
        } else {
            vertical = false;
            slope = (y2 - y1)/(x2 - x1);
            intercept = y2 - slope*x2;
        }
    }

    WallSegment::WallSegment() {}

    WallSegment::~WallSegment() {}

    //Returns 0 if no intersection or parallel lines, 1 else. Considers line information, plus segment limit information.
    bool WallSegment::twoSegmentsIntersection(WallSegment *wsegment1, WallSegment *wsegment2, double& x, double& y) {

        if(twoLinesIntersection(wsegment1, wsegment2, x, y)) {
            double min = WSEGMENT_X1(wsegment1), max = WSEGMENT_X2(wsegment1), aux;
            if(max < min) {
                aux = min; min = max; max = aux;
            }
            if(x > max || x < min)
                return false;
            min = WSEGMENT_X1(wsegment2); max = WSEGMENT_X2(wsegment2);
            if(max < min) {
                aux = min; min = max; max = aux;
            }
            if(x > max || x < min)
                return false;

            return true;
        }

        return false;
    }


    bool WallSegment::inSegmentGivenIntersection(double x, double y) {

        double min, max, aux;

        if(vertical) {
            min = y1; max = y2;
            if(max < min) {
                aux = min; min = max; max = aux;
            }
            if(y > max || y < min)
                return false;
            return true;
        } else if(slope == 0) {
            min = x1; max = x2;
            if(max < min) {
                aux = min; min = max; max = aux;
            }
            if(x > max || x < min)
                return false;
            return true;
        }

        min = x1; max = x2;
        if(max < min) {
            aux = min; min = max; max = aux;
        }
        if(x > max && x < min)
            return false;

        return true;

    }


    //Returns 0 if no intersection or parallel lines, 1 else. Only considers line information, without segment limits.
    bool WallSegment::twoLinesIntersection(WallSegment *wsegment1, WallSegment *wsegment2, double& x, double& y) {
        if((WSEGMENT_IS_VERTICAL(wsegment1) && WSEGMENT_IS_VERTICAL(wsegment2)) || (WSEGMENT_SLOPE(wsegment1) == WSEGMENT_SLOPE(wsegment2)))
            return false;

        if(WSEGMENT_IS_VERTICAL(wsegment1)) {
            x = WSEGMENT_X1(wsegment1);
            y = x*WSEGMENT_SLOPE(wsegment2) + WSEGMENT_INTERCEPT(wsegment2);
            return true;

        } else if(WSEGMENT_IS_VERTICAL(wsegment2)) {
            x = WSEGMENT_X1(wsegment2);
            y = x*WSEGMENT_SLOPE(wsegment1) + WSEGMENT_INTERCEPT(wsegment1);
            return true;
        }

        double
            m1 = WSEGMENT_SLOPE(wsegment1),
            n1 = WSEGMENT_INTERCEPT(wsegment1),
            m2 = WSEGMENT_SLOPE(wsegment2),
            n2 = WSEGMENT_INTERCEPT(wsegment2);

        x = (n1 - n2)/(m2 - m1);
        y = (n1*m2 - n2*m1)/(m2 - m1);

        return true;

    }

    //Returns the pertinence of a coordinate according to a wall segment and stores the value if pertinent
    //  0: if point has no connection with wall segment, according to point orientation(1: vertical, 0:horizontal)
    //  1: if it has connection
    // -1: if it has multiple connections
    // if returns  1: Stores in val1 the value of the intersection
    // if returns -1: Stores in val1 and val2 the extreme values of the segment
    int WallSegment::inWallSegmentInterval(double coord, int orientation, double& val1, double& val2) {

        double min, max;
        val1 = 0;
        val2 = 0;
        if(orientation) {
            if(vertical) {
                if(coord == x1) {
                    val1 = y1;
                    val2 = y2;
                    return -1;
                } else
                    return 0;
            } else { //normal segment
                if(x1 >= x2) {
                    min = x2; max = x1;
                } else {
                    min = x1; max = x2;
                }

                if(coord >= min && coord <= max) { //in interval of interest
                    val1 = coord*slope + intercept;
                    return 1;
                } else
                    return 0;
            }
        } else { //Horizontal case
            if(slope == 0) {
                if(coord == y1) {
                    val1 = x1;
                    val2 = x2;
                    return -1;
                } else
                    return 0;
            } else { //non zero slope segment
                if(y1 >= y2) {
                    min = y2; max = y1;
                } else {
                    min = y1; max = y2;
                }

                if(coord >= min && coord <= max) { //in interval of interest
                    val1 = (coord  - intercept)/slope;
                    return 1;
                } else
                    return 0;
            }
        }
    }

} //end workspace world
