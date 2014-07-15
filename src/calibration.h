#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "QtXml/QDomDocument"
#include <QtXml/QDomElement>
#include <QtXml/QDomText>
#include <QSharedPointer>
#include <QTransform>
#include <string>
#include "world_def.h"

#define SM_CALIB_MATRIX(model)  (model->p_matrix)
#define SM_HOMOGRAPHY(model)  (model->h_matrix)

#define SM_CAMERA_X_FOC_POINT(sm)       ((sm)->camera_focal_point->x)
#define SM_CAMERA_Y_FOC_POINT(sm)       ((sm)->camera_focal_point->y)
#define SM_CAMERA_Z_FOC_POINT(sm)       ((sm)->camera_focal_point->z)

#define SM_CAMERA_X2D_FOC_POINT(sm)       ((sm)->camera_focal_point2D->x)
#define SM_CAMERA_Y2D_FOC_POINT(sm)       ((sm)->camera_focal_point2D->y)

template<typename T>
class point2D;

template<typename T>
class point3D;

//Position relative to blob or other inner model
enum position {
    MiddleCenter = 0x0000, //0000000000000000 : Position at center-middle of blob
    Left         = 0x0001, //0000000000000001 : Position at left-middle of blob
    Right        = 0x0002, //0000000000000010 : Position at right-middle of blob
    Top          = 0x0004, //0000000000000100 : Position at center-top of blob
    Bottom       = 0x0008, //0000000000001000 : Position at center-bottom of blob
    TopLeft      = 0x0005, //0000000000000101 : Position at left-top of blob
    BottomLeft   = 0x0009, //0000000000001001 : Position at left-bottom of blob
    TopRight     = 0x0006, //0000000000000110 : Position at left-top of blob
    BottomRight  = 0x000a, //0000000000001010 : Position at left-bottom of blob
};

//enum for characterising which kind of zone is being used
enum zoneType {
    Z_NONE,
    Z_2D,
    Z_3D,
    Z_H,
};

typedef enum zoneType zoneType;

class SceneModel {
    public:
        QSharedPointer<point3D<double> > camera_focal_point;
        QSharedPointer<point2D<double> > camera_focal_point2D;
        perspective_matrix p_matrix;
        homography_matrix h_matrix;
        QTransform h_trans, h_trans_i;
        bool pmatrix_filled;
        bool hmatrix_filled;
        bool badFocal;

        //Perspective transform context
        std::vector< QSharedPointer<world::ContextObject> > contextObjects;
        //Area of Interest defined with a set of zones
        std::vector< QSharedPointer<world::AOI> > AOIs;
        std::vector< QSharedPointer<world::Area> > Areas;
        std::vector< QSharedPointer<world::Zone> > Zones;
        std::vector< QSharedPointer<world::Wall> > walls;
        std::vector< QSharedPointer<world::Wall2D> > wall2Ds;

        //Homography Zones
        std::vector< QSharedPointer<world::ZoneH> > ZonesH;

        std::vector< QSharedPointer<world::Zone2D> > Zones2D;

        QDomDocument *xmlScene;

        double m_OneMeterRepresentation;
        double m_MaximalObjectSpeed;

        SceneModel();
        ~SceneModel();

        bool readScene(QString fileName);
        bool readParameters(QString fileName);
        bool readObjectModels(QString fileName);

        static position setPosition(std::string pos);
        static void getXY(position pos, double &X, double &Y, Blob b);

        //Compute the image coordinates of a point from its world coordinates. The
        //image coordinates arguments are pointers to double to indicate where to
        //store results. If some of these pointers are NULL, the corresponding
        //results are not computed.
        static int worldToImgCoords(perspective_matrix persp_mat,
                                        double x, double y, double z,
                                        double *X, double *Y);

        //Compute the image coordinates of a point from its homography coordinates.
        //The image coordinates arguments are pointers to double to indicate where to
        //store results. If some of these pointers are NULL, the corresponding
        //results are not computed.
        static int homographyToImgCoords(homography_matrix H,
                                         double X2, double Y2,
                                         double *X, double *Y);



        // Compute the world coordinates of a point from its image coordinates and
        // a given height. The world coordinate arguments are pointers to double
        // to indicate where to store results. If some of these pointers are NULL,
        // the corresponding results are not computed.
        static int imgToWorldCoordsGivenHeight(perspective_matrix persp_mat,
                                                    double X, double Y, double z,
                                                    double *x, double *y);

        // Compute the homography coordinates of a point from its image coordinates.
        // The world coordinate arguments are pointers to double
        // to indicate where to store results. If some of these pointers are NULL,
        // the corresponding results are not computed.
        static int imgToHomographyCoords(homography_matrix H,
                                         double X, double Y,
                                         double *X2, double *Y2);


        bool computeFocalPoint(perspective_matrix pp, int img_width, int img_height,
                                      double *camx, double *camy, double *camz);

        double getLimitHeightForProjectionLine(double x, double y);
        double getLowestImageIntersectingHeight(int W, int H, double x, double y);

};


#endif // CALIBRATION_H
