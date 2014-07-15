#ifndef BLOB_H
#define BLOB_H

#include <QSharedPointer>
#include <iostream>
#include <map>
#include <deque>
#include <list>
#include <vector>
//#include <values.h> values.h? //never heard of it before
#include <cfloat>
#include "parallelpiped.h"

//Forward declaration
class sceneModel;

#define BL_INFINITY             DBL_MAX

class densityData {
  public:
    densityData();
    ~densityData();

    int TP, TN, FP, FN;
    int npixels;
    double hrate, vrate;
    //Added for tracking
    int ncols;
};

class Blob;

//Structure to associate to a blob type probability and reliability estimators on parallelepiped shape for an object type
class Shape3DData {
  public:
    Shape3DData();
    ~Shape3DData();
    Shape3DData *copy();
    Shape3DData& operator=(const Shape3DData& s);
    void copy(Shape3DData *copy_here);
    void setNull(Blob *blob);
    void setNull(ObjectType type, ObjectSubtype subtype, DetectionProblemType dptype, Rectangle<int> *bbox);
    void copyBlobToShape3DData(Blob *blob);
    void copyNoLists(Shape3DData *copy_here);
    void copyShape3DDataOnlyInfo(Shape3DData *copy_here);
    void copyShape3DDataToBlob(Blob *blob);
    double horizontalDistanceToPoint(double X2Dfpoint);
    double verticalDistanceToPoint(double Y2Dfpoint);
    static std::map<ObjectType, Shape3DData>* copyList(std::map<ObjectType, Shape3DData>* list);
    friend std::ostream& operator<<(std::ostream&,const Shape3DData&);

    ObjectType           type;          // blob type
    ObjectSubtype        subtype;       // subtype for postural objects added by marcos
    DetectionProblemType dp_type;     // detection problem type                               */

    BoundingBox bbox;  // 2D bounding box dimensions possibly modified by detection problem resolution */

    Parallelpiped _3Dbbox;       // 3D bounding box                                     */
    densityData ddata;            //Data associated to fitness of 3D bounding box according to moving pixels.
    double w, l, h;               //Dimensions for the 3d bounding box.
    Position3D position3D;   // 3D position of blob base center for a given model type*/
    double Rw, Rl, Rh;            //Reliability measures for each dimension.
    double Pw, Pl, Ph;            //Model probability for each dimension.
    double alpha;                 // Orientation angle (radians) between 3D projection on height 0 of BOTTOM line
                                  // of 2D bbox and line formed by points (x[1],y[1]) and (x[3],y[3]) of 3D bbox.*/
    double P;                     //Basic Probability that the blob belongs to the class
    double R;                     //Global Reliability of the 3d dimensions of the blob
    double PR;                    //Probability Reliability Measure
    double DP;                    //Dimensional Probability Measure
    //Max Distance Factor
    //maxDistanceFactor is a measure of separability of visual evidences (for merging).
    //maxDistanceFactor = 1.0 if visual evidence are intersecting each other.
    //maxDistanceFactor --> 0.0 if visual evidences are too far between them.
    double maxDistanceFactor;

    std::map<ObjectSubtype, Shape3DData> *subtypesList;       //List of subtypes for current type.
};

class Blob {
  public:
    Blob();
    Blob(Blob *);
    Blob(int i_xleft, int i_xright, int i_top, int y_bottom);
    ~Blob();
    void clean();
    Blob *copy();
    Blob *copyWithLists();
    void setRectangle(int i_xleft, int i_xright, int i_ytop, int i_ybottom);
    Blob& operator=(const Blob& b);
    static void copyBlobList(std::vector<Blob>& list, std::vector<Blob>& new_list);
    static Blob *absorbBlob(Blob *blob1, Blob *blob2);
    Blob *verticalSplitBlob(int x2d);
    Blob *horizontalSplitBlob(int y2d);
    static Blob *mergeBlob(Blob *blob1, Blob *blob2);
    void mergeBlob(Blob *blob);
    static Blob *getBiggest2D(std::vector<Blob>& blobs);
    static ObjectType getTypeFromName(std::string name);
    static std::string getNameFromType(ObjectType type);
    static ObjectSubtype getSubtypeFromName(std::string name);
    static std::string getNameFromSubtype(ObjectSubtype subtype);
    static DetectionProblemType getDPTypeFromName(std::string name);
    static std::string getNameFromDPType(DetectionProblemType octype);
    static std::string getDPNameFromTypeDetailed(DetectionProblemType octype);
    static double computeOverlapRatio(Blob *b1, Blob *b2);
    static double computeIntersectRatio(Blob *b1, Blob *b2);
    static double computeOverlapRatioStrict(Blob *b1, Blob *b2);
    static bool isBlob1OverlapBlob2(Blob *b1, Blob *b2);
    bool isPixBelongsToBlob(int x, int y);
    static void whichBlobsPixelBelongsTo(std::list<Blob *>& list, std::vector<Blob>& blobs, int x, int y);
    bool isBlobOccluded();
    bool isBlobTopOccluded();
    bool isBlobBottomOccluded();
    int get3DBBox(SceneModel *smodel, double h, double alpha);
    int compute3DBBoxNormalCase(SceneModel *smodel, double h, double alpha,
                                double V[4][4], int limits[4][4],
                                int eq_relations[4][4], int in_height[4]);

    bool set3DBBox(SceneModel *smodel, double *X, double *Y, double h);
    int compute3DBBoxSpecialCases(SceneModel *smodel, double h, double alpha,
                                  double V[4][4], int limits[4][4], int nlimits[4],
                                  int eq_relations[4][4], int in_height[4]);

    void setInHeightGivenPosition(int position, int *in_height);
    int getPositionRelativeToCamera(SceneModel *smodel);
    int getPositionRelativeToCameraOld(SceneModel *smodel);
    void setReal3DPosition();
    void setEstimated3DPosition(SceneModel* smodel);
    void setPositionAtCenterBottom(SceneModel* smodel);
    static bool isBlob1InsideBlob2(Blob *b1, Blob *b2);
    static bool same2DBlob(Blob *b1, Blob *b2);
    static void set2Dand3Dbboxes(SceneModel *smodel, double *x, double *y,
                                 Rectangle<int> *bb2D, Parallelpiped *bb3D, double h);
    static void orderByProximityToPoint(std::vector<Blob>& blobs, double x, double y);
    double horizontalDistanceToPoint(double X2Dfpoint);
    double verticalDistanceToPoint(double Y2Dfpoint);
    static void orderBlobsAscending(double *values, std::vector<Blob>& blobs, int top, int bottom);
    static int getMiddle(double *values, std::vector<Blob>& blobs, int top, int bottom);
    Shape3DData *getBestS3D(bool &lastUnknown, ObjectType best_type, double classifThreshold);
    static void copyBlobData(Blob *blob1, Blob *blob2);

    friend std::ostream& operator<<(std::ostream&,const Blob&);

    //Variables
    int id;                              // identifier
    int component_label;                 //label assigned at connected components stage.
    ObjectType    type;                 // blob type
    ObjectSubtype subtype;              // Subtype for postural object types
    DetectionProblemType dp_type;       // detection problem type
    BoundingBox bbox;                 // bounding box
    Parallelpiped _3Dbbox;              // bounding box
    densityData ddata;                   // Data associated to fitness of 3D bounding box according to moving pixels.
    char *moving_pixels;                 // List that could contain the moving pixels of a blob.
                                         // It is an array represented as a list, with moving pixels = 1, and background pixels = 0.
                                         // Used in tracking for difered pixel analysis.
    double w, l, h;                      // 3D width
    double alpha;                        // Orientation angle (radians) between 3D X Axis and line formed by
                                         //                   points (x[1],y[1]) and (x[3],y[3]) of 3D bbox.
    int position;
    /* Positions: (Position 4 is Middle-Center and correspond when blob is absolutely between focal-point and ground)
                Left-Front(0)     Middle-Front(1)    Right-Front(2)
                             \           |          /
                                \        |       /
                                   \     |    /
                Left-Center(3)<-----Focal Point----->Right-Center(5)
                                   /     |    \
                                /        |       \
                             /           |          \
                Left-Back(6)       Middle-Back(7)    Right-Back(8)
    */

    bool occlusion;             // true if contained solution in blob corresponds to a solution of the occlusion list
                                // false if contained solution in blob corresponds to a solution of the normal list
    double beta;                // Angle (radians) between 3D projection on height 0 of
                                // CENTER 2D Y of blob line and 3D X axis.
    int beta_direction;         // +1: if positive growth of angle in image plane implies positive growth of angle
                                //     in world xy plane.
                                // -1: if positive growth of angle in image plane implies negative growth of angle
                                //     in world xy plane.

    double gP;                  // Global probability of belonging to the class.
    double gR;                  // Mean reliability of stimated 3D model.
    double gPR;                 //Probability Reliability Measure
    double gDP;                 //Dimensional Probability Measure

    // Dimensions Order: [width3d, height3d, length3d].
    double R[3];                // Reliability measures for each dimension.
    double P[3];                // Model probability for each dimension.

    Position3D position3D; // 3D position of blob

    // list of best 3d associations for each model.
    std::map<ObjectType, Shape3DData> *normal_data;
    // list of best 3d associations for each model, taking into account the posibility of occlusion.
    std::map<ObjectType, Shape3DData> *occ_data;
    // pointer to best 3d model association
    Shape3DData *best_data;

    bool reduced_size;   // true IF BLOB TO LITTLE TO FIT WITH ANY OF THE EXISTING PRE-DEFINED MODELS, ELSE false
    bool excesive_size;  // true IF BLOB TO LITTLE TO FIT WITH ANY OF THE EXISTING PRE-DEFINED MODELS, ELSE false

    int frameNumber;     // Frame Number where blob was processed.

    // Time difference in milliseconds between previous frame and frame where blob was processed.
    int currentTimeDiffMilliSeconds;
    //Max Distance Factor
    //maxDistanceFactor is a measure of separability of visual evidences (for merging).
    //maxDistanceFactor = 1.0 if visual evidence are intersecting each other.
    //maxDistanceFactor --> 0.0 if visual evidences are too far between them.

    double maxDistanceFactor;
};

typedef QSharedPointer<Blob> SpBlob;

// macros for getting blob attributes.
#define BLOB_IDENT(_bl)         ((_bl)->id)

#define BLOB_FRAME_NUMBER(_bl)          ((_bl)->frameNumber)
#define BLOB_TIME_DIFF_MSEC(_bl)        ((_bl)->currentTimeDiffMilliSeconds)

#define BLOB_IS_REDUCED_SIZE(_bl)       ((_bl)->reduced_size)
#define BLOB_IS_EXCESIVE_SIZE(_bl)      ((_bl)->excesive_size)

#define BLOB_BBOX(_bl)          (&((_bl)->bbox))
#define BLOB_3DBBOX(_bl)        (&((_bl)->_3Dbbox))
#define BLOB_DDATA(_bl)         (&((_bl)->ddata))
#define BLOB_MOVING_PIXELS(_bl) ((_bl)->moving_pixels)

#define DDATA_TP(_dd)           (_dd)->TP
#define DDATA_TN(_dd)           (_dd)->TN
#define DDATA_FP(_dd)           (_dd)->FP
#define DDATA_FN(_dd)           (_dd)->FN
#define DDATA_NPIX(_dd)         (_dd)->npixels
#define DDATA_HRATE(_dd)        (_dd)->hrate
#define DDATA_VRATE(_dd)        (_dd)->vrate
#define DDATA_COLS_NUM(_dd)     (_dd)->ncols

#define BLOB_3DBBOX_H(_bl)        (PARALL_H(BLOB_3DBBOX(_bl)))
#define BLOB_3DBBOX_X_i(_bl, _i)  (PARALL_X_i(BLOB_3DBBOX(_bl), _i))
#define BLOB_3DBBOX_Y_i(_bl, _i)  (PARALL_Y_i(BLOB_3DBBOX(_bl), _i))
#define BLOB_3DBBOX_POINT_i(_bl, _i)  (PARALL_POINT_i(BLOB_3DBBOX(_bl), _i))

#define BLOB_3DBBOX_X2D_BASE_i(_bl, _i) (PARALL_X2D_BASE_i(BLOB_3DBBOX(_bl), _i))
#define BLOB_3DBBOX_Y2D_BASE_i(_bl, _i) (PARALL_Y2D_BASE_i(BLOB_3DBBOX(_bl), _i))
#define BLOB_3DBBOX_2D_BASE_POINT_i(_bl, _i) (PARALL_2D_BASE_POINT_i(BLOB_3DBBOX(_bl), _i))

#define BLOB_3DBBOX_X2D_H_i(_bl, _i) (PARALL_X2D_H_i(BLOB_3DBBOX(_bl), _i))
#define BLOB_3DBBOX_Y2D_H_i(_bl, _i) (PARALL_Y2D_H_i(BLOB_3DBBOX(_bl), _i))
#define BLOB_3DBBOX_2D_H_POINT_i(_bl, _i) (PARALL_2D_H_POINT_i(BLOB_3DBBOX(_bl), _i))

#define BLOB_POSITION(_bl)      ((_bl)->position)

#define BLOB_XLEFT(_bl)         (RECT_XLEFT(BLOB_BBOX(_bl)))
#define BLOB_X(_bl)             (BLOB_XLEFT(_bl))
#define BLOB_XRIGHT(_bl)        (RECT_XRIGHT(BLOB_BBOX(_bl)))
#define BLOB_YTOP(_bl)          (RECT_YTOP(BLOB_BBOX(_bl)))
#define BLOB_Y(_bl)             (BLOB_YTOP(_bl))
#define BLOB_YBOTTOM(_bl)       (RECT_YBOTTOM(BLOB_BBOX(_bl)))

#define BLOB_WIDTH(_bl)         (RECT_WIDTH(BLOB_BBOX(_bl)))
#define BLOB_HEIGHT(_bl)        (RECT_HEIGHT(BLOB_BBOX(_bl)))

#define BLOB_XCENTER(_bl)       (BLOB_XLEFT(_bl) + BLOB_WIDTH(_bl)/2)
#define BLOB_YCENTER(_bl)       (BLOB_YTOP(_bl) + BLOB_HEIGHT(_bl)/2)

#define BLOB_TYPE(_bl)          ((_bl)->type)
#define BLOB_SUBTYPE(_bl)       ((_bl)->subtype)
#define BLOB_DP_TYPE(_bl) ((_bl)->dp_type)

#define BLOB_3D_WIDTH(_bl)      ((_bl)->w)
#define BLOB_3D_HEIGHT(_bl)     ((_bl)->h)
#define BLOB_3D_LENGTH(_bl)     ((_bl)->l)

#define BLOB_PW(_bl)    ((_bl)->P[0])
#define BLOB_PH(_bl)    ((_bl)->P[1])
#define BLOB_PL(_bl)    ((_bl)->P[2])
#define BLOB_RW(_bl)    ((_bl)->R[0])
#define BLOB_RH(_bl)    ((_bl)->R[1])
#define BLOB_RL(_bl)    ((_bl)->R[2])

#define BLOB_ALPHA(_bl) ((_bl)->alpha)
#define BLOB_BETA(_bl)  ((_bl)->beta)
#define BLOB_BETA_DIRECTION(_bl)        ((_bl)->beta_direction)

#define BLOB_P(_bl)     ((_bl)->gP)
#define BLOB_R(_bl)     ((_bl)->gR)
#define BLOB_PR(_bl)     ((_bl)->gPR)
#define BLOB_DP(_bl)     ((_bl)->gDP)

#define BLOB_OCCLUSION(_bl)     ((_bl)->occlusion)

#define BLOB_NORMAL_3DDATA(_bl)    ((_bl)->normal_data)
#define BLOB_OCC_3DDATA(_bl)       ((_bl)->occ_data)
#define BLOB_BEST_3DDATA(_bl)      ((_bl)->best_data)

#define BLOB_3D_X(_bl)          ((_bl)->position3D.x)
#define BLOB_3D_Y(_bl)          ((_bl)->position3D.y)
#define BLOB_3D_Z(_bl)          ((_bl)->position3D.z)
#define BLOB_3D_POSITION(_bl)   (&(_bl)->position3D)

/** macro for getting Shape3DData attributes. */
#define S3D_TYPE(_s3d)          ((_s3d)->type)
#define S3D_SUBTYPE(_s3d)       ((_s3d)->subtype)
#define S3D_DP_TYPE(_s3d)      ((_s3d)->dp_type)

#define S3D_BBOX(_s3d)          (&((_s3d)->bbox))
#define S3D_3DBBOX(_s3d)        (&((_s3d)->_3Dbbox))
#define S3D_DDATA(_s3d)         (&((_s3d)->ddata))

#define S3D_XLEFT(_s3d)         (RECT_XLEFT(S3D_BBOX(_s3d)))
#define S3D_X(_s3d)             (S3D_XLEFT(_s3d))
#define S3D_XRIGHT(_s3d)        (RECT_XRIGHT(S3D_BBOX(_s3d)))
#define S3D_YTOP(_s3d)          (RECT_YTOP(S3D_BBOX(_s3d)))
#define S3D_Y(_s3d)             (S3D_YTOP(_s3d))
#define S3D_YBOTTOM(_s3d)       (RECT_YBOTTOM(S3D_BBOX(_s3d)))
#define S3D_WIDTH(_s3d)         (RECT_WIDTH(S3D_BBOX(_s3d)))
#define S3D_HEIGHT(_s3d)        (RECT_HEIGHT(S3D_BBOX(_s3d)))
#define S3D_XCENTER(_s3d)       (S3D_XLEFT(_s3d) + S3D_WIDTH(_s3d)/2)
#define S3D_YCENTER(_s3d)       (S3D_YTOP(_s3d) + S3D_HEIGHT(_s3d)/2)

#define S3D_3DBBOX_H(_s3d)        (PARALL_H(S3D_3DBBOX(_s3d)))
#define S3D_3DBBOX_X_i(_s3d, _i)  (PARALL_X_i(S3D_3DBBOX(_s3d), _i))
#define S3D_3DBBOX_Y_i(_s3d, _i)  (PARALL_Y_i(S3D_3DBBOX(_s3d), _i))
#define S3D_3DBBOX_POINT_i(_s3d, _i)  (PARALL_POINT_i(S3D_3DBBOX(_s3d), _i))

#define S3D_3DBBOX_X2D_BASE_i(_s3d, _i) (PARALL_X2D_BASE_i(S3D_3DBBOX(_s3d), _i))
#define S3D_3DBBOX_Y2D_BASE_i(_s3d, _i) (PARALL_Y2D_BASE_i(S3D_3DBBOX(_s3d), _i))
#define S3D_3DBBOX_2D_BASE_POINT_i(_s3d, _i) (PARALL_2D_BASE_POINT_i(S3D_3DBBOX(_s3d), _i))

#define S3D_3DBBOX_X2D_H_i(_s3d, _i) (PARALL_X2D_H_i(S3D_3DBBOX(_s3d), _i))
#define S3D_3DBBOX_Y2D_H_i(_s3d, _i) (PARALL_Y2D_H_i(S3D_3DBBOX(_s3d), _i))
#define S3D_3DBBOX_2D_H_POINT_i(_s3d, _i) (PARALL_2D_H_POINT_i(S3D_3DBBOX(_s3d), _i))

#define S3D_3D_X(_s3d)          ((_s3d)->position3D.x)
#define S3D_3D_Y(_s3d)          ((_s3d)->position3D.y)
#define S3D_3D_Z(_s3d)          ((_s3d)->position3D.z)
#define S3D_3D_POSITION(_s3d)   (&(_s3d)->position3D)

#define S3D_W(_s3d)     ((_s3d)->w)
#define S3D_H(_s3d)     ((_s3d)->h)
#define S3D_L(_s3d)     ((_s3d)->l)

#define S3D_PW(_s3d)    ((_s3d)->Pw)
#define S3D_PH(_s3d)    ((_s3d)->Ph)
#define S3D_PL(_s3d)    ((_s3d)->Pl)

#define S3D_RW(_s3d)    ((_s3d)->Rw)
#define S3D_RH(_s3d)    ((_s3d)->Rh)
#define S3D_RL(_s3d)    ((_s3d)->Rl)

#define S3D_R(_s3d)     ((_s3d)->R)
#define S3D_P(_s3d)     ((_s3d)->P)
#define S3D_PR(_s3d)     ((_s3d)->PR)
#define S3D_DP(_s3d)     ((_s3d)->DP)

#define S3D_ALPHA(_s3d) ((_s3d)->alpha)

#define S3D_SUBTYPES_LIST(_s3d) ((_s3d)->subtypesList)

typedef Shape3DData *s3ddata_t;
typedef densityData *ddata_t;


class blobBuffer {
    private:
        unsigned int maxLength;

    public:
        std::deque<SpBlob> trackedBlobs;
        blobBuffer(int);
        ~blobBuffer();
        std::deque<SpBlob>::iterator begin();
        std::deque<SpBlob>::iterator end();
        std::deque<SpBlob>::reverse_iterator rbegin();
        std::deque<SpBlob>::reverse_iterator rend();
        SpBlob operator[](int);
        unsigned int size();
        SpBlob back();
        void clear();
        void insert(Blob *);
        void resetMaxLength(int);
        void copyBlobs(blobBuffer *);
        friend std::ostream& operator<<(std::ostream&,const blobBuffer&);
};


#endif // BLOB_H
