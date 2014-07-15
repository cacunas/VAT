#ifndef __RELIABILITY_CLASSIFICATION__
#define __RELIABILITY_CLASSIFICATION__

//#include <values.h> values.h? //never heard of it before
#include "Classification.h"
#include "intervals.h"
#include "world_def.h"
#include "MobileObject.h"
#include "Datapool.h"

#define RC_INFINITY DBL_MAX

//Class created to manage information concerning the influence of walls of context objects limiting points of the 2D bounding box.
//Used to determine the limit of possible growth of a limit of the 2D bounding box (LEFT, RIGHT, TOP or BOTTOM).
class inWall {
    private:
        world::Wall2D *wall;
        double enters;
        double leaves;
        int entering_type;
        int leaving_type;
  
    public:
        inWall();
        inWall(world::Wall2D *i_wall, double i_enters, int i_enter_type, double i_leaves, int i_leave_type);
        ~inWall();
        world::Wall2D *getWall();
        double getEntering();
        double getLeaving();
        int getEnteringType();
        int getLeavingType();
        //To store entering information as leaving, and viceversa.
        void reverseOrder();
        inWall& operator=(const inWall& inwall);
};

class ReliabilityClassification {
    public:
        //Variables for the user defined static parameters:

        //Variables for sub-parameters of ordering by pixels.
        static int RC_MAX_PIXELS;     //Maximal number of pixels to be analyzed
        static int RC_MIN_PIXELS;     //Minimal number of pixels to be analyzed
        static double m_pixelDensity; //Desired rate of pixels to analyze in blob.

        bool initialized;
        bool imageWorkingAreaNotComputed;


        double ALPHA_STEP; //Step to advance in the search of parallelpiped orientation alpha (radians).
        double H_STEP_CM;  //Step to advance in the search of parallelpiped height h (centimeters. Ex: for "5" centimeters, "5" if context defined in centimeters, "0.05" if context defined in meters).
        int DIM2D_STEP; //pixels step for exploration of new solutions in case of detection problem.

        bool m_orderByDensity; //Flag that activates ordering of best solutions by maximal number of (TP + TN)/Num_Analyzed_Blob_Pixels. TP (true positives) is the number of moving pixels inside the
                               //contour of the parallelpiped. TN (true negatives) is the number of non-moving pixels outside the contour of the parallelpiped.
        bool m_treatBorderOcclusion; //Flag that activates the treatment of possible occlusions in the image borders.
        bool m_treatContextObjectOcclusion; //Flag that activates the treatment of possible occlusions with context objects.
        bool m_treatWallCoherence; //Flag that activates the verification of walls near to the generated parallelpiped, to check if parallelpiped base is not crossing any wall.

        int m_rcntop; //Top solutions list length.

        //Selection criteria for fitting 3d model to a class (intra) and to select the best between clases (inter)
        //Available:
        //                  RC_PROBABILITY:             Multiplication of probabilities for parallelpiped dimensions.
        //                  RC_RELIABILITY_PROBABILITY: Sum of the reliability times probability for each dimension, all divided by the sumation of reliabilities.
        //                  RC_RELIABILITY:             Mean reliability of dimensions
        //                  RC_DIMENSIONAL:             Sum of the length times probability for each dimension, all divided by the sumation of length of dimensions.
        classificationCriteria m_rcIntraCriteria;
        classificationCriteria m_rcInterCriteria;

        //Variables for sub-parameters of wall coherency checking:
        double m_cuttingAreaRate;      //Maximal rate of area the parallelpiped base allowed to be cutted by a limiting wall.
        double m_wallInsideLengthRate; //Maximal cutting length rate of the wall to be inside the parallelpiped base.

        int RC_PROXIMITY_PIXELS; //Variable for sub-parameter of context object occlusion. It represents the limit to consider is a blob is near a context object.

        double m_maxRoofAndBaseIntersectionRatio, m_maxBaseAndBaseIntersectionRatio;

        double m_minPixelCoverageRatio;

        //Variables:
        //Datapool
        Datapool *m_data;

        //Scene model
        SceneModel *m_context;
        //Pointer to current foreground result
        QImage *m_foreground;

        //currentRealDimensionsOfBlobBBox
        Rectangle<int> realBBox;

        //Classification Threshold for intra criteria
        double m_classifThres;
        //Classification Threshold for inter criteria
        double m_classifThresInter;

        //Flag for knowing if parameters setting has been already performed
        bool m_parametersSet;

        //Set pertinent walls for each image limit, to be used when determining the maximal possible growth of 2D bounding box limits, in the case of possible border occlusion.
        std::multimap<DetectionProblemType, world::Wall2D *> m_pertinent_walls;

        //container of the models found in the context file
        std::map<ObjectType, SpModelInterface> m_mapModels;
        std::map<ObjectType, bool> sizeOkForAnalysis;
        std::map<ObjectType, std::map<ObjectSubtype, bool> > subSizeOkForAnalysis;
        //classification map by pure probability
        std::map<int, std::map<ObjectType, double> > m_mapClassif;

        //For Rigid Object Models
        //Maps to store the best solutions with and without occlusion.
        std::map<ObjectType, std::deque<Shape3DData *> > m_top;
        std::map<ObjectType, std::deque<Shape3DData *> > m_topocc;
        std::map<ObjectType, int> m_ntop;
        std::map<ObjectType, int> m_ntopocc;

        //Best solution without considering min and max limits, for each possible blob type
        std::map<ObjectType, s3ddata_t> best_valid;
        std::map<ObjectType, bool> there_is_best_valid;
        s3ddata_t best_considering_alpha;
        s3ddata_t best_considering_h;
        bool best_by_alpha_found, best_by_h_found;
        double bestAlphaDistance, bestDimDistance;
        double currentDimDistance;
        //For Postural Object Models
        bool currentRigid;
        //Maps to store the best solutions with and without occlusion.
        std::map<ObjectType, std::map<ObjectSubtype, std::deque<Shape3DData *> > > m_sub_top;
        std::map<ObjectType, std::map<ObjectSubtype, std::deque<Shape3DData *> > > m_sub_topocc;
        std::map<ObjectType, std::map<ObjectSubtype, int> > m_sub_ntop;
        std::map<ObjectType, std::map<ObjectSubtype, int> > m_sub_ntopocc;
        //Best solution without considering min and max limits, for each possible blob type
        std::map<ObjectType, std::map<ObjectSubtype, s3ddata_t> > sub_best_valid;
        std::map<ObjectType, std::map<ObjectSubtype, bool> > sub_there_is_best_valid;

        //Current Object Model information
        SpModelInterface m_currentModel;
        ObjectType m_modelId;
        ObjectSubtype m_subModelId;

        //Variables used for Tracking
        double *g_postureMinw, *g_postureMinl, *g_postureMinh;
        double *g_postureMaxw, *g_postureMaxl, *g_postureMaxh;
        int Winitial, Hinitial;
        double alphaInit, alphaMobile, alphaChange, alphaVar, hInit, dChange, hVar, hMin, hMax, hMobile, wMobile, lMobile;

        Interval interval_hmobile;

        //VARIABLES FOR CONTEXT OBJECTS OCCLUSION
        //List of not virtual walls of context objects
        std::deque<world::Wall2D *> real_walls;
        //List of objects near to blob
        std::deque<world::ContextObject *> near_objects;
        //List of objects far from blob
        std::deque<world::ContextObject *> not_near_objects;
        //Map of objects and the type of possible occlusion they produce
        std::map<world::ContextObject *, DetectionProblemType> object_occlusion_type;
        //When blob size is extended new objects can be near.
        std::deque<world::ContextObject *> near_objects_extended;
        //Maps between objects and a map of the analyzed points of the blob with a list of the walls that can limit the growth of a blob.
        //Four maps defined, one for each object occlusion type.

        std::map<world::ContextObject *, std::map<int, std::deque<inWall> > > enteredWallsForPointLeft;
        std::map<world::ContextObject *, std::map<int, std::deque<inWall> > > enteredWallsForPointRight;
        std::map<world::ContextObject *, std::map<int, std::deque<inWall> > > enteredWallsForPointTop;
        std::map<world::ContextObject *, std::map<int, std::deque<inWall> > > enteredWallsForPointBottom;

        //Maps between objects and a map of the analyzed points of the blob with its corresponding value.
        //Two maps are defined. One for the X 2D coordinates and the other for the Y 2D coordinates.
        std::map<world::ContextObject *, std::map<int, double> > analyzedPointsForObjectX;
        std::map<world::ContextObject *, std::map<int, double> > analyzedPointsForObjectY;


        //Used in Coherency checking, this map stores the potentially conflicting walls with parallelpiped base.
        std::deque<world::WallSegment *> annoyingWallSegments;

        double m_HToleranceCoeff, m_VToleranceCoeff; //Tolerance to determine the limits for considering an object out of the FOV.
        double imgCamx, imgCamy;                     //Projection of the camera 3D position in the floor of the scene, in image coordinates.
        double camx, camy, camz;                     //3D position of the focal point of the camera.
        int m_XMinWA, m_XMaxWA, m_YMinWA, m_YMaxWA;  //Real image limits, according to the tolerance values.
        double H, W;                                 //2D Dimensions of the blob.
        double Hmax, Wmax;                           //Maximal possible blob dimensions, when occlusion occurs.
        double borderMinLeft, borderMaxRight, borderMinTop, borderMaxBottom; //Limits imposed by border occlusion analysis to the value for each 2D occlusion direction.
        double objectMinLeft, objectMaxRight, objectMinTop, objectMaxBottom; //Limits imposed by context object occlusion analysis to the value for each 2D  occlusion direction.
        double OcclusionMinLeft, OcclusionMaxRight, OcclusionMinTop, OcclusionMaxBottom; //Global limits to the value for each 2D occlusion direction.
        bool possible_occlusion;                     //Flag to signal if any occlusion can happen.
        double model_hmin, model_hmax, model_hmean,  //Currently analyzed expected object model limits and mean for each of its dimensions.
               model_lmin, model_lmax, model_lmean,
               model_wmin, model_wmax, model_wmean;

        Interval interval_modelw, interval_modelh, interval_modell; //Intervalar representation of expected object model dimension limits.

        double alpha, sina, cosa, beta;                             //Variables associated to parallelpiped orientation.
        int beta_direction;                                         //Direction of plane xy of the 3D referential of the scene. (Right or inverted coordinate system)
        //Counters for display and to detect certain cases where it is not necessary to continue the search of solutions in the case of occlusion.
        int with_sol_counter, models_calculated_counter, global_models_calculated_counter, valid_models_counter, models_accepted_after_walls_checking;


        //Values for determining the case of a 3D bounding box. Case 0 is the normal one, and Cases 1 to 4 are special cases.
        // Case 0:       Normal case. Four verteces of the parallelpiped touching a 2D bounding box limit Each of them touch a different 2D limit.
        // Case 1, 2, 3: A parallelpiped vertex in a corner of the bounding box (touching two 2D limits), and two other touching the remaining two 2D limits. The position of the other verteces touching
        //               the 2D limits with respect to the vertex that touches two limits, defines the Case, between 1, 2, or 3.
        // Case 4:       Two verteces touching two 2D limits each.
        // a, b, c, d represent the bounding limits of 3D points in the special cases. sp_var stores the position of the first variable
        // involved in a doble limit (point in a corner of 2D blob).
        int Case, a, b, c, d, sp_var;

        //Values for boosting 3D parallelpiped model search.
        //blob level values, calculated using blob 2D dimensions and information associated to the relative position of the blob with respect to the camera.
        double V[4][4];
        double
          p00, p01, p02, p03,
          p10, p11, p12, p13,
          p20, p21, p22, p23;
        double
          k1,  k2,  k3,  k4,  k5,  k6,  k7,  k8,  k9,  k10,
          k11, k12, k13, k14, k15, k16, k17, k18, k19, k20,
          k21, k22, k23, k24, k25, k26, k27, k28, k29, k30,
          k31, k32, k33, k34;
        double Vl0, Vl1, Vl2, Vb0, Vb1, Vb2, Vr0, Vr1, Vr2, Vt0, Vt1, Vt2;
        double ml0, ml1, mb0, mb1, mr0, mr1, mt0, mt1;

        //For new simpler classification
        double
          KK1,  KK2,  KK3,  KK4,
          KK5,  KK6,  KK7,  KK8,
          KK9,  KK10, KK11, KK12,
          KK13, KK14, KK15, KK16;
       double
          LL1,  LL2,  LL3,  LL4,
          LL5,  LL6,  LL7,  LL8,
          LL9,  LL10, LL11, LL12,
          LL13, LL14, LL15;

       double DD, MM1, MM2, MM3, MM4, MM5, MM6;

        //alpha angle level values:
        double K[22], L[5], C[16];

        //Equation constants for calculating w and l in terms of h, and viceversa
        double d1, d2, d3, d4, d5, d6, d7, d8;
        double wa, wb, wc, la, lb, lc;

        int
          position, //Position relative to 2D image projection of camera focal point projected in the xy plane of the world referential of the scene.
          limits[4][4], // limits[3DBB_POINT][2D_LIMIT] value is 1 if the parallelpiped 3DBB_POINT is bounded by the 2D_LIMIT.
                        //2D_LIMIT = { 0=left, 1=bottom, 2=right, 3= top}
                        //3DBB_POINT = {0, 1, 2, 3}
          nlimits[4],   // nlimits[2D_LIMIT] value correspond to the number of 3DBB_POINTs bounded by 2D_LIMIT. One value different from one correspond to a
                        // special case in 3D bounding box calculation (normally one 3D point bounded by one 2D limit)
          in_height[4], // in_height[2D_LIMIT] value is 1 if 3DBB_POINT(s) bounded by 2D_LIMIT are bounded in the highest point (h), else 0 (if bounded in their base point (height=0)).
          BlobLimits[4],// value for each limit in blob in 2D image coordinates (2D_LIMIT = { 0=left, 1=bottom, 2=right, 3= top} );
          varlimrel[4]; // varlimrel[2D_LIMIT] = 3DBB_POINT indicating the 3DBB_POINT index bounded by the 2D_LIMIT.

        int eq_relations[4][4];     //Given the existence of two types of equations to relate two variables
                                    //this matrix stablishes the types of relations between variables.
                                    //0: no relation, 1:equation type 1, 2:equation type 2.
        bool normalised90;
        bool normalised180;
      protected:
        //Flag for defining if the occlusion list will be created for a blob
        bool someOcclusionSolutionInserted, someNormalSolutionInserted, aoi_in;
        //Set object type, occlussion type and 3D bounding box for all blobs
         void constructClassifMap(std::vector<Blob>&, QImage *);

      public:

        ReliabilityClassification();

        ReliabilityClassification(Datapool *i_data);

        double convertAlpha(double original_alpha);

        double NormalizeAngle(double alpha);

        //Set pertinent analysis window of the image
        void computeImageWorkingArea(QImage *i_segm);

        void initBlob3DData();

        //Decide for the right limits for all possible growths of a blob, according to all types of occlusion detected and to physical limits imposed by current
        //analyzed expected object model.
        void adjustBlobLimitsByOcclusion(Blob *blob);

        //Determine if there is border occlusion and return the border occlusion type, according to analysis window of the image.
        DetectionProblemType isBlobOnImageBorder(Blob *blob);

        //Set object type, occlussion type and 3D bounding box for a blob
        void setBlob3DFacts(Blob *blob, QImage *i_segm);

        //Set in blob the data concerning the last 3D parallelpiped analysis performed.
        void setLastBlobData(double h, double l, double w, double alpha, Blob *blob, bool _90_rotated);

        //Returns the obtained measured probability value for a possible solution to the parallelpiped determination problem and stores the solution in the list of top
        //solutions if the new solution is good enough in terms of the probabilistic measure.
        double set_possible_solution(double w, double l, double h, Blob *blob, bool rotated, bool normal);

        //Search for parellelepiped solution for different sizes of a blob, according to the detected types of possible occlusions.
        void fillOcclusionList(Blob *blob);

        //Search for parallelpiped solutions, changing orientation alpha, for the occlusion case.
        double static_occlusion_search_alpha(Blob *);

        //Considering a fixed angle alpha and fixed blob dimensions, search in the space of possible heights h, the best solution, which
        //accomplish with the limits imposed by the expected object model and the objects in the scene.
        double search_solution_by_height(Blob *, interval_t, interval_t, bool);

        //Search for parallelpiped solutions, changing orientation alpha, for the occlusion case.
        double static_occlusion_search_alpha(Blob *, std::map<ObjectType, SpModelInterface>::iterator);

        //Considering a fixed angle alpha and fixed blob dimensions, search in the space of possible heights h, the best solution, which
        //accomplish with the limits imposed by the expected object model and the objects in the scene.
        double search_solution_by_height(Blob *, std::map<ObjectType, SpModelInterface>::iterator, interval_t, interval_t, bool);

        //add a class model in the set of all models which have to be classified
        void addModel(ObjectType, SpModelInterface);

        //update of classification map
        void removeBlob(Blob *);

        //get map of classification
        inline const std::map<int, std::map<ObjectType, double> >& getClassifMap() const {return m_mapClassif;}

        //get map of models
        inline const std::map<ObjectType, SpModelInterface>& getModelsMap() const {return m_mapModels;}

        SpModelInterface getModelFromType(ObjectType);

        //Search for the best occluding or not occluding parallelpiped solutions, searching the optimal for different parallelpiped orientation angles and heights, and
        //different 2D blob dimensions in the case of possible occlusion.
        void top_probabilities_and_reliabilities(Blob *blob);

        //Init data structures on Module Load level
        virtual void init_data();

        //Top classification map management
        void top_init();

        //Eliminates the elements presents in the top lists.
        void reset_top();

        //Frees the top lists
        void free_top();


        //Check if the quality of a solution allows it to enter to the top list.
        bool can_be_inserted_top (Blob *blob, bool normalList);

        //Insert a solution in a top list.
        s3ddata_t insert_top(Blob *blob, bool normalList);

        //Function to display the content of a top list.
        void top_output();

        void computeGeometricFacts(Blob *blob);

        virtual ~ReliabilityClassification();

        virtual void updateScoreForType(Blob *i_blob1, ObjectType i_blobType, double i_score);

        //Set final object type and detection problem type for a blob
        void buildClassifTable(Blob *blob);

        //Set all pertinent calculations that won't change for the given blob 2D dimensions
        void RC_set_3D_bbox_blob_level_data(Blob *blob);

        //Set the information considering parallelpiped orientation alpha and blob 2D dimensions as constants.
        void RC_set_3D_bbox_initial_alpha_level_data(double alpha);

        //Set the information considering parallelpiped orientation alpha and blob 2D dimensions as constants knowing the type (Case) of parallelpiped
        //solutions that can be obtained. Normally a parallelpiped has one vertex point on each blob dimension, but there are some degenerate cases
        //with some vertexes in more than one blob dimension.
        void RC_set_3D_bbox_alpha_level_data();

        //Calculate the parallelpiped 3D points according to height and pre-calculated constants.
        int RC_compute_blob_3Dbbox(Blob *blob, double h);

        //Set vertical and horizontal advancement in pixel analysis in term of number of pixels, using parameters pre-defined by the end-user.
        static void setPixelRates(int bwidth, int bheight, double *horRate, double *verRate);

        //Used after 3D bounding box construction, to store the number of moving or not moving pixels inside or outside the generated parallelpiped.
        void setPixelAnalysis(Blob *blob, ddata_t ddata, parallelpiped_t _3Dbbox);

        //Used to store moving pixel blob information, for utilisation in later frames.
        void setExtraPixelInfo(Blob *blob, ddata_t ddata, QImage *i_segm);

        //Generate the polygon representing the silhouette of a parallelpiped.
        static polygon2D<double> *set_polygon_from_3Dbb(parallelpiped_t _3Dbbox, int position);

        //Gets the index of the nearest base point to a blob 2D limit, between the base points lying between the nearest and the farest base points (middle points).
        static int get_nearest_from_middle_points(parallelpiped_t _3Dbbox, int limit);

        //This function orders the top solutions lists in terms of the number of pixels inside the generated parallelpiped.
        void putBestByPixels(bool occlusion);

        //Calculates the criteria of comparison between solutions. It corresponds to True positives (moving points inside the parallelpiped) + True negatives (not moving
        //points out of the parallelpiped), divided by the total number of analyzed pixels.
        double densityDistance(ddata_t ddata);

        double check_walls(Blob *blob, DetectionProblemType octype);

        //Checks if an object is close enough of a context object. If it is the case, it searches to determine the correct context object occlusion type
        //It also sets the annoying walls of the object, for coherency between 3D parallelpiped base and these walls
        DetectionProblemType isBlobOccludedByStaticObject(Blob *blob);

        //Stores the best solution regardless if the solution does not accomplishes with dimensional limits imposed by the currently analyzed expected object model.
        //This result is used as an starting point by the occlusion treatement part if no real solution have been found.
        double set_best_without_model_limits(Blob *blob, double w, double l, double h, double alpha);

        //Returns the probability value for certain parallelpiped dimensions.
        double getProbability(double h, double l, double w);

        //Classify the context walls according to their position with respect to blob.
        void init_pertinent_occlusion_walls();

        //Set the pertinent maximal vertical blob limits (LEFT and RIGHT), according to limiting context walls
        void check_walls_ver(Blob *, DetectionProblemType);

        //Set the pertinent maximal horizontal blob limits (TOP and BOTTOM), according to limiting context walls
        void check_walls_hor(Blob *, DetectionProblemType);

        //Checks if the base of a calculated parallelpiped does not collide with a wall in the scene. Pertinent context walls and walls of context objects are analyzed.
        bool parallelpiped_base_inside_pertinent_walls(Blob *, double, double);

        //Returns the maximal limit (Wmax or Hmax) for the size of a blob, according to the maximal limits
        //of dimensions of the expected object in the scen currently processed.
        double limits_by_max_possible_blob(Blob *, DetectionProblemType);

        //Obtain the k distant 3D point from other point, according to the slope, the intercept of the line where the new point must be and the direction with respect to
        //the initial point.
        void get_xy_k_distant_from_init_point(double *, double *, double, double, double, double, double, double);

        //Compute 3D height h  possible values, given the 3D width w and pre-calculated constants.
        void RC_compute_h_from_w(double, double *, double *);

        //Compute 3D height h possible values, given the 3D length l and pre-calculated constants.
        void RC_compute_h_from_l(double, double *, double *);

        //Compute 3D width w, given the 3D height h and pre-calculated constants.
        int RC_compute_w_from_h(double, double *);

        //Compute 3D length l, given the 3D height h and pre-calculated constants.
        int RC_compute_l_from_h(double, double *);

        //Get 3D height h, given the 3D width w and pre-calculated constants, if there is a valid solution.
        void RC_get_h_from_w(double, double *, double);

        //Get 3D height h, given the 3D length l and pre-calculated constants, if there is a valid solution.
        void RC_get_h_from_l(double, double *, double);

        //Determine which Case of parallelpiped solution is, according to the position of parallelpiped vertexes in the 2D blob limits.
        int RC_set_3Dbbox_case_and_variable_indexes();

        //Calculate the four classificationCriteriaals where there can be geometrically valid parallelpipeds to be generated.
        int calculate_hintervals(interval_t, interval_t);
        int calculate_hintervals_old(interval_t, interval_t);

        //Checks if exists vertical or horizontal intersection between two pairs of intervals
        bool thereIsIntersection(interval_t, interval_t, interval_t, interval_t);

        //Perform the analysis of nine points in the blob to determine the correct blob occlusion type and the wall segments which will limit the possible growth
        //of blob dimensions in the case of context object detection problems
        DetectionProblemType improveObjectAnalysis(world::ContextObject *, DetectionProblemType, Blob *, interval_t, interval_t);

        //Checks if all possible base points analyzed are inside a context object
        bool all_possible_base_inside(bool [][3], int);

        //This function returns the number of points inside a context object. Nine points are analyzed, for three coordinates of x2D and three of y2D.
        int determine_points_inside_context_object(world::ContextObject *, double *, double *, bool [][3]);

        //Clear the lists of walls of analyzed blob points which limit the growth of the 2D bounding box in the case of context object occlusion.
        void clearWallsInPoint();

        //Set the wall segments that can limit the vertical growth of the blob in case of context object occlusion, for each analyzed point of the blob.
        void checkPointsInWallVer(world::ContextObject *, double[], double[], DetectionProblemType, world::Wall2D *);

        //Set the wall segments that can limit the horizontal growth of the blob in case of context object occlusion, for each analyzed point of the blob.
        void checkPointsInWallHor(world::ContextObject *, double[], double[], DetectionProblemType, world::Wall2D *);

        //Set the segments intersecting the analyzed points in the different directions of possible growth of the blob in case of context object occlusion.
        void checkPointsInRoof(world::ContextObject *, double[], double[], QSharedPointer< polygon2D<double> >, DetectionProblemType);

        //Function to check if intersection between a 2D image coordinates vertical or horizontal line and a 2D segment exists. It also stores the pertinent intersection point
        //of the free coordinate (if line is vertical the free coordinate is y2d, else the free corrdinate is y2d).
        bool getIntersectionWithSegment(double, double, double, world::WallSegment *, bool, double&);

        //Use the information obtained from the limiting walls for analyzed points, to determine if the blob has a real possible of being occluded.
        DetectionProblemType correctBlobDetectionProblemType(world::ContextObject *, double[], double[], DetectionProblemType);

        //Calculates the limiting blob growth bounds for all pertinent context object occlusion cases
        void setObjectOcclusionLimit(Blob *, DetectionProblemType, bool);

        //Set the potencially colliding context object walls for certain blob 2D dimension, for checking when parallelpiped base is obtained.
        void setAnnoyingWallsForContextObject(Blob *, world::ContextObject *, bool);

        //Set the potencially colliding context walls for certain blob 2D dimension, for checking when parallelpiped base is obtained.
        void setAnnoyingWallsForContextWalls(Blob *, bool);

        //Checks if an object is close enough of a context object and store it in lists of near or not near objects.
        void setNearObjects(Blob *);

        //Add the information of context objects near to the maximized version of the blob, according to calculated dimension growth limits,
        //when occlusion situations occur.
        void setMaxBlobNearObjects();

        //Checks if a blob is inside a visible context wall.
        bool blobOverContextWalls(Blob *);

        //Get the projection in a height (the 3D maximal expected object model) for the farest point of the intersection between a segment and a vertical
        //segment in y2D image coordinates. This projection imposes the softer limit over an analyzed point x2D in image coordinates. Used in the case of
        //horizontal context object occlusion.
        double getX2DInHeight(std::map<int, double>, world::Wall2D *, double, bool);

        //Get the projection in a height (the 3D maximal expected object model) for the farest point of the intersection between a segment and a horizontal
        //segment in x2D image coordinates. This projection imposes the softer limit over an analyzed point y2D in image coordinates. Used in the case of
        //vertical context object occlusion.
        double getY2DInHeight(std::map<int, double>, world::Wall2D *, double, bool);

        //Returns true if we have found solutions but all of them have collided with a wall.
        bool noWallsValidatedSolution();

        void checkBlobSize(Blob *blob);
        //Version for Reliability Tracking
        bool checkBlobSize(int &reduced, int &excesive, s3ddata_t s3d, ObjectType type, int position, DetectionProblemType occtype);

        void checkSubModelsSize( int &reduced, int &excesive, SpModelInterface objectModel, ObjectType modelType, Blob *blob);
        //Version for Reliability Tracking
        void checkSubModelsSize( int &reduced, int &excesive, SpModelInterface objectModel, ObjectType modelType, s3ddata_t s3d, int position, DetectionProblemType occtype);

        int checkIfBlobIsReducedSize(Blob *blob, double hmin, double lmin, double wmin);
        //Version for Reliability Tracking
        int checkIfBlobIsReducedSize(s3ddata_t s3d, double hmin, double lmin, double wmin, int position);

        int checkIfBlobIsExcesiveSize(Blob *blob, double hmax, double lmax, double wmax);
        //Version for Reliability Tracking
        int checkIfBlobIsExcesiveSize(s3ddata_t s3d, double hmax, double lmax, double wmax, int position);

        //Quicksort
        static void orderBlobsAscending(double *values, Blob **blobs, int top, int bottom);

        static int getMiddle(double *values, Blob **blobs, int top, int bottom);

        //Functions used by tracking and pre-tracking modules
        Shape3DData *getMostCoherentDataFromMobileAndBBoxLimit(MobileObject *mobile, std::map<ObjectType, Shape3DData> *&normal,
                                                                                  std::map<ObjectType, Shape3DData> *&occ, Blob *blob);
        void getMostCoherentDataFromMobile(s3ddata_t bests3d, MobileObject *mobile, DetectionProblemType occtype, Blob *blob);

        void setPostureLimits(ObjectType type);

        bool reliability_tracking_search_alpha(Blob *blob);

        double search_solution_by_height_for_tracking(Blob *blob, interval_t interval_h, double previous_best);

        double set_possible_solution_for_tracking(double w, double l, double h, Blob *blob, double current_best);

        int calculate_hintervals_for_tracking(interval_t interval_h_0);
        int calculate_hintervals_for_tracking_old(interval_t interval_h_0);

        bool set_best_valid_alpha_for_tracking();

        void limits_by_mean_model_values(double *mean_limits, double *starting_angles, Blob *blob, DetectionProblemType octype);

        void fillOcclusionListWithStartingPoints(Blob *blob);

        void static_occlusion_search_2D(double *mean_limits, double *starting_angles, Blob *blob, DetectionProblemType occtype, double dim_limit);

        double static_occlusion_search_alpha_with_starting_point(Blob *blob, double& starting_alpha);

        void limits_by_mean_model_values_bottom_occlusion(double *mean_limits, double *mean_left_limits, double *mean_right_limits,
                                                          double *starting_angles, Blob *blob, bool with_left_occlusion,
                                                          Rectangle<int> *mean_bboxes, parallelpiped_t meanparallelpipeds);

        void fillOcclusionListWithStartingPoints2(Blob *blob);

        int RC_set_3Dbbox_case_and_variable_indexes_simpler(Blob *blob);
        int RC_compute_blob_3Dbbox_get_case(Blob *blob);

        void RC_set_3D_bbox_alpha_level_data_get_case();

        void RC_set_3D_bbox_alpha_level_data_simpler();

        int calculate_hintervals_simpler(interval_t interval_h_0, interval_t interval_h_90);

        int RC_compute_blob_3Dbbox_simpler(Blob *blob, double h);

        void static_occlusion_bottom_search_2D_in_corner(double *mean_limits, double *mean_left_limits, double *mean_right_limits,
                                                         double *starting_angles, Blob *blob, DetectionProblemType ver_occtype,
                                                         DetectionProblemType hor_occtype, double ver_dim_limit, double hor_dim_limit);

        bool not_over_an_object(parallelpiped_t par, bool not_occluded);

        bool covering_bbox(Blob *blob);

        bool covering_bbox(int blob_position, Blob *blob, ddata_t ddata, parallelpiped_t _3Dbbox);

        void setPixelAnalysis(int blob_position, Blob *blob, ddata_t ddata, parallelpiped_t _3Dbbox);

        void prepareBlobForRClassification(Blob *blob, QImage *i_segm);

        void setDPForBlob(Blob *blob, QImage *i_segm);

        s3ddata_t getMostLikelyDataFromMobile(Blob *blob, ObjectType type);

        s3ddata_t getMostLikelyDataForType(Blob *blob);

        s3ddata_t generateS3DListsForBlob(Blob *blob, ObjectType analyzed_type);

        s3ddata_t addBestDataForType(bool normal, Blob *blob, ObjectType analyzed_type, std::map<ObjectType, Shape3DData> *s3d_list, s3ddata_t data);

        std::map<ObjectType, Shape3DData> *getBestOnesList(bool normal, Shape3DData **best_s3d, double *best, bool *exists3d, Blob *i_blob);

        void setExtraPixelInfo(Blob *blob, QImage *i_segm);

        bool positiveAngleDirection(double beta, double angle);

        bool allowed_base_dimensions(double w, double l);

        bool setParameters(QDomNode& config);

        bool defaultParameters();

        void classify(std::vector<Blob>& i_blobs, QImage *i_segm);

        bool parallelpipedBaseOnAOIGround(Parallelpiped *par, SceneModel *context);

        bool blobOnAOIGround(Blob *i_blob, SceneModel *context);

        void assignTypeToBlob(Blob *i_blob);

        bool prepareRun();

        bool init();

        bool checkIfBetter(double *best, Shape3DData *s3d);

        bool changeBlobTypeToUnknown(Blob *i_blob);

};

typedef QSharedPointer<ReliabilityClassification> SpReliabilityClassification;

#endif


