#ifndef _WORLD_DEF_H_
#define _WORLD_DEF_H_

#include "common.h"
#include <string>
#include <vector>
#include <QSharedPointer>
#include <QString>
#include <QDomNode>
#include <QPainter>

class SceneModel;
class Area;
class Blob;

template<typename T>
class polygon2D;

template<typename T>
class polygon3D;

template<typename T>
class Rectangle;

template<typename T>
class point2D;

template<typename T>
class point3D;

namespace world {

class AOI {
    public:
        AOI();
        ~AOI();
        bool pointInside(double x, double y);
        void buildPoly3DOnGround(std::vector< QSharedPointer<Area> > &areas);
        bool blobInsideGround3D(Blob *blob);
        int id;
        QString name;
        QSharedPointer < polygon3D<double> > poly3DOnGround;
        QSharedPointer < polygon2D<double> > poly2DOnGround;
        double  height;
        QSharedPointer < polygon2D<double> > outlineOnImg;
};

class Zone {
    public:
        Zone();
        ~Zone();
        bool rectangleInside(Rectangle<int> *rect);
        bool pointInZone(double x, double y);

        int id;
        QString name;
        QSharedPointer < polygon3D<double> > outline3D;
        QSharedPointer < polygon2D<int> > outline;
};

/*enum zone3DType {
    Z3D_polygon,
    Z3D_ellipse
};

typedef enum zone3DType zone3DType;
*/

enum zone2DType {
    Z2D_polygon,
    Z2D_ellipse
};

typedef enum zone2DType zone2DType;

class ZoneH {
    public:
        ZoneH();
        virtual ~ZoneH()=0;
        virtual bool rectangleInside(Rectangle<int> *rect)=0;
        virtual bool pointInZone(double x, double y)=0;
        virtual bool setParameters(QDomNode &params, homography_matrix h_matrix)=0;
        virtual bool draw(QPainter &p, QColor &color, QTransform &h_trans)=0;
        virtual double distanceToCenter(double x, double y)=0;

        int id;
        QString name;
        zone2DType type;
};


class ZoneHPolygon: public ZoneH {
    public:
        ZoneHPolygon();
        ~ZoneHPolygon();
        bool rectangleInside(Rectangle<int> *rect);
        bool pointInZone(double x, double y);
        bool setParameters(QDomNode &params, homography_matrix h_matrix);
        bool draw(QPainter &p, QColor &color, QTransform &h_trans);
        double distanceToCenter(double x, double y);

        //Homography
        QSharedPointer < polygon2D<double> > outline2D;
        //Image
        QSharedPointer < polygon2D<int> > outline;
};

class ZoneHEllipse: public ZoneH {
    public:
        ZoneHEllipse();
        ~ZoneHEllipse();
        bool rectangleInside(Rectangle<int> *rect);
        bool pointInZone(double x, double y);
        bool setParameters(QDomNode &params, homography_matrix h_matrix);
        bool draw(QPainter &p, QColor &color, QTransform &h_trans);
        double distanceToCenter(double x, double y);

        //Homography
        double x, y, rx, ry;
};


class Zone2D {
    public:
        Zone2D();
        virtual ~Zone2D()=0;
        virtual bool rectangleInside(Rectangle<int> *rect) = 0;
        virtual bool pointInZone(double x, double y) = 0;
        virtual bool setParameters(QDomNode &params)=0;
        virtual bool draw(QPainter &p, QColor color)=0;

        int id;
        QString name;
        zone2DType type;
};

class Zone2DPolygon: public Zone2D {
    public:
        Zone2DPolygon();
        ~Zone2DPolygon();
        bool rectangleInside(Rectangle<int> *rect);
        bool pointInZone(double x, double y);
        bool setParameters(QDomNode &params);
        bool draw(QPainter &p, QColor color);

        //Image
        QSharedPointer < polygon2D<int> > outline;
};

class Zone2DEllipse: public Zone2D {
    public:
        Zone2DEllipse();
        ~Zone2DEllipse();
        bool rectangleInside(Rectangle<int> *rect);
        bool pointInZone(double x, double y);
        bool setParameters(QDomNode &params);
        bool draw(QPainter &p, QColor color);

        //Image
        int x, y, rx, ry;
};


class Area {
    public:
        Area();
        ~Area();

        bool pointInside(double x, double y);
        bool zoneInside(QString name);

        QString name;
        std::vector< QSharedPointer<Zone> > visibleParts;
};

class Wall {
    public:
        Wall();
        ~Wall();
        //Calculates parameters for 3D line equation calculated from wall limiting points
        void setWallAsLineEquation();

        int id;
        QString name;
        QSharedPointer< point3D<double> > startPoint;
        QSharedPointer< point3D<double> > endPoint;
        double height;
        bool solid; //true: wall is real
                    //false: wall is virtual (just for building context object)
        bool segmentIsFunction; // if a -> infinite => segmentIsFunction = false and x = constant for all y
                                // else segmentIsFunction = true and y = a*x + b
        double a, b;            // Segment equation parameters
};

//Represent a limiting wall segment of a wall in 2D image coordinates, classifying line segments according to their 3D nature.
//if inHeight is 2, the point (x1,y1) will correspond to the projection of the 3D point on the floor and (x2,y2) to the one in height.
class WallSegment {
    public:
        WallSegment();
        ~WallSegment();
        void setWallSegment(double x1, double y1, double x2, double y2, int in_height);

        //Returns the pertinence of a coordinate according to a wall segment and stores the value if pertinent
        // false: if point has no connection with wall segment, according to point orientation
        int inWallSegmentInterval(double coord, int orientation, double& val1, double& val2);

        //Returns false if no intersection or parallel lines, true else. Considers line information, plus segment limit information.
        static bool twoSegmentsIntersection(WallSegment *wsegment1, WallSegment *wsegment2, double& x, double& y);

        //Returns false if no intersection or parallel lines, true else. Only considers line information, without segment limits.
        static bool twoLinesIntersection(WallSegment *wsegment1, WallSegment *wsegment2, double& x, double& y);

        //Returns false if no intersection, true else. Evaluates segment limits given an intersection point.
        bool inSegmentGivenIntersection(double x, double y);

        double x1;
        double y1;
        double x2;
        double y2;
        double slope;
        double intercept;
        bool vertical;  // true: When slope tends to infinite;
                        // false: Normal slope
        int inHeight;   // 0: Segment 2D corresponding to the one on the floor of the 3D scene.
                        // 1: Segment 2D corresponding to the one on the maximal height of wall in 3D coordinates.
                        // 2: Segment 2D corresponding to the one going from the floor to the maximal height of wall in 3D coordinates.
};

//Defines the 2D image transform to wall at different 3D heights h 
//with              y2d = x2d * a(h) + b(h)
// and:
//                  a = (K[0]*z + K[1])/(K[4]*z + K[5]);
//                  b = (K[2]*z + K[3])/(K[4]*z + K[5]);
//The wall projection at height h have as 2d image points:
//                 X2d1 = (pp2*z + A[0])/(pp10*z + C[0]);
//                 Y2d1 = (pp6*z + B[0])/(pp10*z + C[0]);
//                 X2d2 = (pp2*z + A[1])/(pp10*z + C[1]);
//                 Y2d2 = (pp6*z + B[1])/(pp10*z + C[1]);
//, where ppX is the value in position X in the perspective matrix                  
class Wall2D {
    public:
        Wall2D();
        Wall2D(Wall *wall, SceneModel *smodel);
        ~Wall2D();

        //Valid for pinhole camera model, using a perspective matrix
        //Returns the parameters a and b for a image plane line y2d = a*x2d + b, given wall and h
        bool get2DWallLine(double h, double *a, double *b);

        //Returns the limit 2D image points p1 and p2, given wall and h
        void get2DwallLimitPoints(double h, perspective_matrix persp_mat, point2D<double> *p1, point2D<double> *p2);

        //Returns true if point (x2d,y2d) is inside of pertinent zone according to wall and height h
        bool pointIsPertinent(double h, perspective_matrix persp_mat, double x2d, double y2d);

        //Returns true if point p is inside of pertinent zone according to wall, for a point in the floor
        bool pointInFloorIsPertinent(perspective_matrix persp_mat, double x2d, double y2d);

        int id;
        QString name;
        bool visible; //false: Not visible
                      //true: Visible (or partially visible)
        int discriminationType; //Defines the way of considering a point inside or outside camera's pertinent zone
            //0: y2d - x2d*a - b <= 0
            //1: y2d - x2d*a - b >= 0
        DetectionProblemType occPertinance;
        QSharedPointer < polygon2D<double> > outlineOnImage;
        double max_h;
        double A[2],B[2],C[2];   //Parameters for wall 2D limit points given h
        double K[6];             //Parameters for wall 2D line equation parameters a and b, given h
        WallSegment wsegments[4];     //Wall representation as a set of four limiting segments
        WallSegment base3Dsegment;    //Segment representing the base line (h = 0) in 3D scene coordinates
};

class ContextObject {
    public:
        ContextObject();
        ~ContextObject();
        int setObjectOutlineFromWallList();
        int setObjectWallListFromOutline();

        int id;
        QString name;
        double height;		     // the height above the plane
        QSharedPointer< polygon3D<double> > outline;   // Representation of context object by list of 3D points

        // Representation of objects by walls (uses the possibility of virtual walls for hollow objects)
        std::vector< QSharedPointer<Wall> > wallList;
        // 2D Representation of walls which form the context object
        std::vector< QSharedPointer<Wall2D> > wall2DList;

        QSharedPointer < polygon2D<double> > outline2DOnImage;    // projection of object on the image, including its limiting 2D bbox
        QSharedPointer < polygon2D<double> > poly2DOfObjectBase;  // projection of the object base on the image
        QSharedPointer < polygon2D<double> > poly2DOfObjectRoof;  // projection of the object roof on the image
        QSharedPointer < polygon2D<double> > poly2DOfOutline;     // 2D polygon of the base of the 3D polygon in world coordinates.

        bool hollow; //true: object is a bunch of walls
        bool noRoof; //true: object has no roof
};

class Camera {
    public:
        Camera();
        ~Camera();
        int id;       // use for interface with the wrapper
        QString name;
        QSharedPointer < point3D<double> > location;
        QSharedPointer < point3D<double> > focalPoint;     // 3D focal point for calibrated scene
        QSharedPointer < point2D<double> > focalPoint2D;   // focal point projected on the ground of the scene.
};

//MACROS

#define AREA_NAME(ar)		 ((ar)->name)
#define AREA_NTH_PART(ar, n)     ((ar)->visibleParts[n])

#define AOI_NAME(aoi)			((aoi)->name)
#define AOI_ID(aoi)			((aoi)->id)
#define AOI_POLY3D_ON_GROUND(aoi)	((aoi)->poly3dOnGround)
#define AOI_POLY2D_ON_GROUND(aoi)       ((aoi)->poly2dOnGround)
#define AOI_OUTLINE_ON_IMG(aoi)		((aoi)->outlineOnImg)
#define AOI_HEIGHT(aoi)			((aoi)->height)

#define ZONE_ID(zn)			((zn)->id)
#define ZONE_NAME(zn)			((zn)->name)
#define ZONE_OUTLINE(zn)		((zn)->outline)
#define ZONE_OUTLINE_2D(zn)		((zn)->outline2D)
#define ZONE_OUTLINE_3D(zn)		((zn)->outline3D)

#define WALL_ID(wl)                     ((wl)->id)
#define WALL_NAME(wl)			((wl)->name)
#define WALL_START_POINT(wl)		((wl)->startPoint)
#define WALL_START_POINT_X(wl)          ((wl)->startPoint->x)
#define WALL_START_POINT_Y(wl)          ((wl)->startPoint->y)
#define WALL_START_POINT_Z(wl)          ((wl)->startPoint->z)
#define WALL_END_POINT(wl)              ((wl)->endPoint)
#define WALL_END_POINT_X(wl)            ((wl)->endPoint->x)
#define WALL_END_POINT_Y(wl)            ((wl)->endPoint->y)
#define WALL_END_POINT_Z(wl)            ((wl)->endPoint->z)
#define WALL_HEIGHT(wl)                 ((wl)->height)
#define WALL_IS_SOLID(wl)               ((wl)->solid)
#define WALL_SEGMENT_IS_FUNCTION(wl)    ((wl)->segmentIsFunction)
#define WALL_A(wl)                      ((wl)->a)
#define WALL_B(wl)                      ((wl)->b)

#define WALL2D_ID(wl)                   ((wl)->id)
#define WALL2D_NAME(wl)                 ((wl)->name)
#define WALL2D_VISIBLE(wl)              ((wl)->visible)
#define WALL2D_DIS_TYPE(wl)             ((wl)->discriminationType)
#define WALL2D_OCC_PERT(wl)             ((wl)->occPertinance)
#define WALL2D_PARAM_A(wl)              ((wl)->A)
#define WALL2D_PARAM_B(wl)              ((wl)->B)
#define WALL2D_PARAM_C(wl)              ((wl)->C)
#define WALL2D_PARAM_K(wl)              ((wl)->K)
#define WALL2D_PARAM_A_i(wl,i)          ((wl)->A[i])
#define WALL2D_PARAM_B_i(wl,i)          ((wl)->B[i])
#define WALL2D_PARAM_C_i(wl,i)          ((wl)->C[i])
#define WALL2D_PARAM_K_i(wl,i)          ((wl)->K[i])
#define WALL2D_MAXH(wl)                   ((wl)->max_h)
#define WALL2D_OUTLINE_ON_IMAGE(wl)     ((wl)->outlineOnImage)
#define WALL2D_SEGMENT(wl,i)            (&((wl)->wsegments[i]))
#define WALL2D_SEGMENT_BASE3D(wl)       (&((wl)->base3Dsegment))

#define WSEGMENT_X1(ws)                 ((ws)->x1)
#define WSEGMENT_Y1(ws)                 ((ws)->y1)
#define WSEGMENT_X2(ws)                 ((ws)->x2)
#define WSEGMENT_Y2(ws)                 ((ws)->y2)
#define WSEGMENT_SLOPE(ws)              ((ws)->slope)
#define WSEGMENT_INTERCEPT(ws)          ((ws)->intercept)
#define WSEGMENT_IS_VERTICAL(ws)        ((ws)->vertical)
#define WSEGMENT_IN_HEIGHT(ws)          ((ws)->inHeight)

#define WOBJECT_ID(wo)                  ((wo)->id)
#define WOBJECT_NAME(wo)		((wo)->name)
#define WOBJECT_HEIGHT(wo)		((wo)->height)
#define WOBJECT_OUTLINE(wo)             ((wo)->outline)
#define WOBJECT_WALLS_LIST(wo)	        (&(wo)->wallList)
#define WOBJECT_WALLS_2D_LIST(wo)	(&(wo)->wall2DList)
#define WOBJECT_2D_OF_OUTLINE(wo)      ((wo)->poly2DOfOutline)
#define WOBJECT_OUTLINE2D_ON_IMAGE(wo) ((wo)->outline2DOnImage)
#define WOBJECT_BASE2D_ON_IMAGE(wo)    ((wo)->poly2DOfObjectBase)
#define WOBJECT_ROOF2D_ON_IMAGE(wo)    ((wo)->poly2DOfObjectRoof)
#define WOBJECT_IS_HOLLOW(wo)           ((wo)->hollow)
#define WOBJECT_NO_ROOF(wo)             ((wo)->noRoof)

#define CAMERA_ID(ca)	            ((ca).id)
#define CAMERA_NAME(ca)             ((ca)->name)
#define CAMERA_LOCATION(ca)	    (&(ca).location)
#define CAMERA_X_LOCATION(ca)	    ((ca).location.x)
#define CAMERA_Y_LOCATION(ca)	    ((ca).location.y)
#define CAMERA_Z_LOCATION(ca)	    ((ca).location.z)

} //end namespace world

#endif /* _WORLD_DEF_H_ */
