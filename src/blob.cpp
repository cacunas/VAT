#include <stdio.h>
#include <math.h>
//#include "multi_cntx.h"
#include "blob.h"
#include "parallelpiped.h"
#include <iostream>
#include "calibration.h"
//#include "common.h"

Blob::Blob() {
    normal_data = occ_data = NULL;
    best_data = NULL;
    moving_pixels = NULL;
    component_label = -1;
    clean();
}

Blob::Blob(Blob *blob) {
    *this = *blob;
}


Blob::Blob(int i_xleft, int i_xright, int i_top, int y_bottom) {
    normal_data = occ_data = NULL;
    best_data = NULL;
    moving_pixels = NULL;
    clean();
    setRectangle(i_xleft, i_xright, i_top, y_bottom);
    component_label = -1;
}

Blob::~Blob() {
    if(normal_data != NULL) {
        delete normal_data;
        normal_data = NULL;
    }
    if(occ_data != NULL) {
        delete occ_data;
        occ_data = NULL;
    }
    if(moving_pixels != NULL) {
        delete[] moving_pixels;
        moving_pixels = NULL;
    }
}

void Blob::clean() {
    if(normal_data != NULL)
        delete normal_data;
    if(occ_data != NULL)
        delete occ_data;
    if(moving_pixels != NULL)
        delete[] moving_pixels;
    memset(this ,0, sizeof(Blob));
    this->type = UNKNOWN;
    this->subtype = ST_UNKNOWN;
    dp_type = MM_DP_NONE;
    maxDistanceFactor = 1.0;
    BLOB_R(this) = 1.0;
}

Blob *Blob::copy() {
    Blob *new_blob = new Blob();
    memcpy(new_blob, this, sizeof(Blob));
    new_blob->moving_pixels = NULL;
    new_blob->best_data = NULL;
    return new_blob;
}

Blob *Blob::copyWithLists() {
    Blob *new_blob = new Blob();
    memcpy(new_blob, this, sizeof(Blob));
    if(normal_data != NULL) {
        new_blob->normal_data = new std::map<ObjectType, Shape3DData>();
        *(new_blob->normal_data) = *normal_data;
    }
    if(occ_data != NULL) {
        new_blob->occ_data = new std::map<ObjectType, Shape3DData>();
        *(new_blob->occ_data) = *occ_data;
    }
    if(moving_pixels != NULL) {
        new_blob->moving_pixels = new char[sizeof(moving_pixels)];
        memcpy(new_blob->moving_pixels, moving_pixels, sizeof(moving_pixels));
    }
    if(best_data == NULL)
        new_blob->best_data = NULL;
    else if(best_data->dp_type == MM_DP_NONE)
        new_blob->best_data = &((*normal_data)[best_data->type]);
    else
        new_blob->best_data = &(*occ_data)[best_data->type];

    return new_blob;
}

void Blob::setRectangle(int i_xleft, int i_xright, int i_ytop, int i_ybottom) {
    bbox.xleft = i_xleft;
    bbox.xright = i_xright;
    bbox.ytop = i_ytop;
    bbox.ybottom = i_ybottom;
    bbox.width = i_xright - i_xleft + 1;
    bbox.height = i_ybottom - i_ytop + 1;
}

Blob& Blob::operator=(const Blob& b) {
    memcpy(this, &b, sizeof(Blob));
    if(b.normal_data != NULL) {
        normal_data = new std::map<ObjectType, Shape3DData>();
        *normal_data = *b.normal_data;
    }
    if(b.occ_data != NULL) {
        occ_data = new std::map<ObjectType, Shape3DData>();
        *occ_data = *b.occ_data;
    }
    if(b.moving_pixels != NULL) {
        moving_pixels = new char[sizeof(b.moving_pixels)];
        memcpy(moving_pixels, b.moving_pixels, sizeof(moving_pixels));
    }
    if(b.best_data == NULL)
        best_data = NULL;
    else if(b.best_data->dp_type == MM_DP_NONE)
        best_data = &((*normal_data)[b.best_data->type]);
    else
        best_data = &(*occ_data)[b.best_data->type];

    return *this;
}

void Blob::copyBlobList(std::vector<Blob>& list, std::vector<Blob>& new_list) {
    int i = 0;
    new_list.reserve(list.size());
    std::vector<Blob>::iterator iter, it_end = list.begin();
    for(iter = list.begin(); iter != it_end; iter++)
        new_list[i++] = *iter;
}

Blob *Blob::absorbBlob(Blob *blob1, Blob *blob2) {
    if(blob1 == NULL) return NULL;
    if(blob2 == NULL) return NULL;

    BLOB_BBOX(blob1)->absorbRectangle(BLOB_BBOX(blob2));

    if (BLOB_MOVING_PIXELS(blob1) != NULL)
      delete BLOB_MOVING_PIXELS(blob1);

    BLOB_MOVING_PIXELS(blob1) = NULL;

    return blob1;
}

//Split a blob, applying a vertical cut in x2d x coordinate of the image. 
//Returns right side blob
Blob *Blob::verticalSplitBlob(int x2d) {

    if (x2d <= BLOB_XLEFT(this) || x2d >= BLOB_XRIGHT(this))
        return NULL;

    Blob *second_split = this->copy();
    BLOB_XRIGHT(this) = x2d;
    BLOB_XLEFT(second_split) = x2d + 1;
    BLOB_WIDTH(this) = x2d - BLOB_XLEFT(this) + 1;
    BLOB_WIDTH(second_split) = BLOB_XRIGHT(second_split) - x2d;

    BLOB_MOVING_PIXELS(second_split) = NULL;

    return second_split;
}

//Split a blob, applying a horizontal cut in y2d y coordinate of the image.
//Returns bottom side blob
Blob *Blob::horizontalSplitBlob(int y2d) {

    if (y2d <= BLOB_YTOP(this) || y2d >= BLOB_YBOTTOM(this))
        return NULL;

    Blob *second_split = this->copy();
    BLOB_YBOTTOM(this) = y2d;
    BLOB_YTOP(second_split) = y2d + 1;
    BLOB_HEIGHT(this) = y2d - BLOB_YTOP(this) + 1;
    BLOB_HEIGHT(second_split) = BLOB_YBOTTOM(second_split) - y2d;

    return second_split;
}


Blob *Blob::mergeBlob(Blob *blob1, Blob *blob2) {
    if((blob1 == NULL) && (blob2 == NULL))
        return NULL;
    if(blob1 == NULL)
        return blob2->copy();
    if(blob2 == NULL)
        return blob1->copy();

    if(blob1 == blob2)
        return blob1->copy();

    Blob *merge = new Blob();

    // update new blob, bbox
    BLOB_XLEFT(merge)  = std::min(BLOB_XLEFT(blob1), BLOB_XLEFT(blob2));
    BLOB_XRIGHT(merge) = std::max(BLOB_XRIGHT(blob1), BLOB_XRIGHT(blob2));
    BLOB_YTOP(merge)   = std::min(BLOB_YTOP(blob1), BLOB_YTOP(blob2));
    BLOB_YBOTTOM(merge)= std::max(BLOB_YBOTTOM(blob1), BLOB_YBOTTOM(blob2));
    BLOB_WIDTH(merge)  = BLOB_XRIGHT(merge) - BLOB_XLEFT(merge) + 1;
    BLOB_HEIGHT(merge) = BLOB_YBOTTOM(merge)- BLOB_YTOP(merge) + 1;

    BLOB_TIME_DIFF_MSEC(merge) = BLOB_TIME_DIFF_MSEC(blob1);
    BLOB_MOVING_PIXELS(merge) = NULL;

    BLOB_R(merge) = (BLOB_R(blob1) + BLOB_R(blob2)) / 2.0;

    return merge;
}

//Merges blobs and store result in this
void Blob::mergeBlob(Blob *blob) {

  if( blob == NULL || this == blob)
    return;

  // update blob 1 bbox
  BLOB_XLEFT(this)  = std::min(BLOB_XLEFT(this), BLOB_XLEFT(blob));
  BLOB_XRIGHT(this) = std::max(BLOB_XRIGHT(this), BLOB_XRIGHT(blob));
  BLOB_YTOP(this)   = std::min(BLOB_YTOP(this), BLOB_YTOP(blob));
  BLOB_YBOTTOM(this)= std::max(BLOB_YBOTTOM(this), BLOB_YBOTTOM(blob));
  BLOB_WIDTH(this)  = BLOB_XRIGHT(this) - BLOB_XLEFT(this) + 1;
  BLOB_HEIGHT(this) = BLOB_YBOTTOM(this)- BLOB_YTOP(this) + 1;

  BLOB_R(this) = (BLOB_R(this) + BLOB_R(blob)) / 2.0;


  if (BLOB_MOVING_PIXELS(this) != NULL)
    delete BLOB_MOVING_PIXELS(this);
  BLOB_MOVING_PIXELS(this) = NULL;
}


Blob *Blob::getBiggest2D(std::vector<Blob>& blobs) {

    if(blobs.empty())
        return NULL;

    std::vector<Blob>::iterator iter = blobs.begin(), it_end = blobs.end();
    Blob *current, *biggest = &*iter;
    double c_area, b_area = BLOB_WIDTH(biggest) * BLOB_HEIGHT(biggest);

    for(iter++;iter != it_end;iter++) {
        current = &*iter;
        c_area = BLOB_WIDTH(current) * BLOB_HEIGHT(current);
        if(c_area > b_area) {
            biggest = current;
            b_area = c_area;
	}
    }
    return biggest;
}

ObjectType Blob::getTypeFromName(std::string name) {
    if(name == "PERSON")
        return PERSON;
    else if(name == "A_GROUP")
        return A_GROUP;
    else if(name == "PERSON_GROUP")
        return PERSON_GROUP;
    else if(name == "CONTEXT_OBJECT")
        return CONTX_OBJECT;
    else if(name == "VEHICLE")
        return VEHICLE;
    else if(name == "NOISE")  /* new */
        return NOISE;
    else if(name == "CROWD")  /* new */
        return CROWD;
    else if(name == "CAR")
        return CAR;
    else if(name == "TRUCK")
        return TRUCK;
    else if(name == "MOTORCYCLE")
        return MOTORCYCLE;
    else if(name == "CHILD")
        return CHILD;
    else if (name == "PARKED_VEHICLE")
        return PARKED_VEHICLE;
    else if(name == "FACE")
        return FACE;
    else if(name == "TWO-PERSONS")
        return TWOPERSONS;
    else if(name == "THREE-PERSONS")
        return THREEPERSONS;
    else if(name == "ANY2D")
        return ANY2D;
    else if(name == "ANY3D")
        return ANY3D;
    else if(name == "UNKNOWN")
        return UNKNOWN;
    else {
          std::cout << "Warning: unknown requested object type " << name << std::endl;
          return UNKNOWN;
    }
}

std::string Blob::getNameFromType(ObjectType type) {

    if(type == PERSON)
        return "PERSON";
    else if(type == A_GROUP)
        return "A_GROUP";
    else if(type == PERSON_GROUP)
        return "PERSON_GROUP";
    else if(type == CONTX_OBJECT)
        return "CONTEXT";
    else if(type == VEHICLE)
        return "VEHICLE";
    else if(type == NOISE)
        return "NOISE";
    else if(type == CROWD)
        return "CROWD";
    else if(type == CAR)
        return "CAR";
    else if(type == TRUCK)
        return "TRUCK";
    else if(type == MOTORCYCLE)
        return "MOTORCYCLE";
    else if(type == CHILD)
        return "CHILD";
    else if (type == PARKED_VEHICLE)
        return "PARKED_VEHICLE";
    else if(type == FACE)
        return "FACE";
    else if(type == TWOPERSONS)
        return "TWO-PERSONS";
    else if(type == THREEPERSONS)
        return "THREE-PERSONS";
    else if(type == ANY2D)
        return "ANY2D";
    else if(type == ANY3D)
        return "ANY3D";
    else
        return "UNKNOWN";
}

ObjectSubtype Blob::getSubtypeFromName(std::string name) {
    if(name == "NO_SUBTYPES")
        return ST_NO_SUBTYPES;  //Used by rigid objects
    else if(name == "STANDING")
        return ST_STANDING;
    else if(name == "CROUCHING")
        return ST_CROUCHING;
    else if(name == "LYING")
        return ST_LYING;
    else if(name == "UNKNOWN")
        return ST_UNKNOWN;

    std::cout << "Warning: undefined expected object subtype in context file:" << name << ". It must be defined in blob.h" << std::endl;
        return ST_UNDEFINED;
}

std::string Blob::getNameFromSubtype(ObjectSubtype subtype)
{
if(subtype == ST_NO_SUBTYPES) //Used by rigid objects
    return "GLOBAL";
else if(subtype == ST_STANDING)
    return "STANDING";
else if(subtype == ST_CROUCHING)
    return "CROUCHING";
else if(subtype == ST_LYING)
    return "LYING";
else if(subtype == ST_UNKNOWN)
    return "UNKNOWN";

return "UNDEFINED";
}

DetectionProblemType Blob::getDPTypeFromName(std::string name) {
    if (name == "MM_DP_NONE")
    return MM_DP_NONE;
    else if (name == "MM_CAM_OCCL_BOTTOM")
	return MM_CAM_OCCL_BOTTOM;
    else if (name == "MM_CAM_OCCL_TOP")
	return MM_CAM_OCCL_TOP;
    else if (name == "MM_CAM_OCCL_LEFT")
	return MM_CAM_OCCL_LEFT;
    else if (name == "MM_CAM_OCCL_RIGHT")
	return MM_CAM_OCCL_RIGHT;
    else if (name == "MM_CAM_OCCL_BOTTOM_LEFT")
	return MM_CAM_OCCL_BOTTOM_LEFT;
    else if (name == "MM_CAM_OCCL_BOTTOM_RIGHT")
	return MM_CAM_OCCL_BOTTOM_RIGHT;
    else if (name == "MM_CAM_OCCL_TOP_LEFT")
	return MM_CAM_OCCL_TOP_LEFT;
    else if (name == "MM_CAM_OCCL_TOP_RIGHT")
	return MM_CAM_OCCL_TOP_RIGHT;
    else if (name == "MM_AOI_OUTSIDE")
	return MM_AOI_OUTSIDE;
    else if (name == "MM_AOI_REINSIDE")
	return MM_AOI_REINSIDE;
    else if (name == "MM_OBJECT_BOTTOM")
	return MM_OBJECT_BOTTOM;
    else if (name == "MM_OBJECT_TOP")
	return MM_OBJECT_TOP;
    else if (name == "MM_OBJECT_INSIDE")
	return MM_OBJECT_INSIDE;
    else if (name == "MM_OBJECT_RIGHT")
	return MM_OBJECT_RIGHT;
    else if (name == "MM_OBJECT_LEFT")
	return MM_OBJECT_LEFT;
    else {
        std::cout << "Warning: unknown detection problem type: " << name << std::endl;
    return MM_DP_NONE;
    }
}

//Simple transform from detection problem type to text
std::string Blob::getNameFromDPType(DetectionProblemType dptype) {

  if(dptype == MM_DP_NONE)
    return "MM_DP_NONE";
  else if(dptype == MM_CAM_OCCL_BOTTOM)
    return "MM_CAM_OCCL_BOTTOM";
  else if(dptype == MM_CAM_OCCL_TOP)
    return "MM_CAM_OCCL_TOP";
  else if(dptype == MM_CAM_OCCL_LEFT)
    return "MM_CAM_OCCL_LEFT";
  else if(dptype == MM_CAM_OCCL_RIGHT)
    return "MM_CAM_OCCL_RIGHT";
  else if(dptype == MM_CAM_OCCL_BOTTOM_LEFT)
    return "MM_CAM_OCCL_BOTTOM_LEFT";
  else if(dptype == MM_CAM_OCCL_BOTTOM_RIGHT)
    return "MM_CAM_OCCL_BOTTOM_RIGHT";
  else if(dptype == MM_CAM_OCCL_TOP_LEFT)
    return "MM_CAM_OCCL_TOP_LEFT";
  else if(dptype == MM_CAM_OCCL_TOP_RIGHT)
    return "MM_CAM_OCCL_TOP_RIGHT";
  else if(dptype == MM_AOI_OUTSIDE)
    return "MM_AOI_OUTSIDE";
  else if(dptype == MM_AOI_REINSIDE)
    return "MM_AOI_REINSIDE";
  else if(dptype == MM_OBJECT_BOTTOM)
    return "MM_OBJECT_BOTTOM";
  else if(dptype == MM_OBJECT_TOP)
    return "MM_OBJECT_TOP";
  else if(dptype == MM_OBJECT_INSIDE)
    return "MM_OBJECT_INSIDE";
  else if(dptype == MM_OBJECT_RIGHT)
    return "MM_OBJECT_RIGHT";
  else if(dptype == MM_OBJECT_LEFT)
    return "MM_OBJECT_LEFT";
  else if(dptype == MM_OBJECT_BOTTOM_LEFT)
    return "MM_OBJECT_BOTTOM_LEFT";
  else if(dptype == MM_OBJECT_BOTTOM_RIGHT)
    return "MM_OBJECT_BOTTOM_RIGHT";
  else if(dptype == MM_OBJECT_TOP_LEFT)
    return "MM_OBJECT_TOP_LEFT";
  else if(dptype == MM_OBJECT_TOP_RIGHT)
    return "MM_OBJECT_TOP_RIGHT";
  
  return "MM_OCCL_OTHER";
  
}

std::string Blob::getDPNameFromTypeDetailed(DetectionProblemType dptype) {
  if(dptype == MM_DP_NONE)
    return "OCC:NONE";

  std::string result("OCC:");

  if(dptype & MM_CAM_OCCL_BOTTOM) {
    result += result.size() != 4 ? "-CAM_BOT" : "CAM_BOT";
    dptype = (DetectionProblemType)(dptype - MM_CAM_OCCL_BOTTOM);
  }
  if(dptype & MM_CAM_OCCL_TOP) {
    result += result.size() != 4 ? "-CAM_TOP" : "CAM_TOP";
    dptype = (DetectionProblemType)(dptype - MM_CAM_OCCL_TOP);
  }
  if(dptype & MM_CAM_OCCL_LEFT) {
    result += result.size() != 4 ? "-CAM_LEF" : "CAM_LEF";
    dptype = (DetectionProblemType)(dptype - MM_CAM_OCCL_LEFT);
  }
  if(dptype & MM_CAM_OCCL_RIGHT) {
    result += result.size() != 4 ? "-CAM_RIG" : "CAM_RIG";
    dptype = (DetectionProblemType)(dptype - MM_CAM_OCCL_RIGHT);
  }
  if(dptype & MM_AOI_OUTSIDE) {
    result += result.size() != 4 ? "-AOI_OUT" : "AOI_OUT";
    dptype = (DetectionProblemType)(dptype - MM_AOI_OUTSIDE);
  }
  if(dptype & MM_AOI_REINSIDE) {
    result += result.size() != 4 ? "-AOI_REI" : "AOI_REI";
    dptype = (DetectionProblemType)(dptype - MM_AOI_REINSIDE);
  }
  if(dptype & MM_OBJECT_BOTTOM) {
    result += result.size() != 4 ? "-OBJ_BOT" : "OBJ_BOT";
    dptype = (DetectionProblemType)(dptype - MM_OBJECT_BOTTOM);
  }
  if(dptype & MM_OBJECT_TOP) {
    result += result.size() != 4 ? "-OBJ_TOP" : "CAM_TOP";
    dptype = (DetectionProblemType)(dptype - MM_OBJECT_TOP);
  }
  if(dptype & MM_OBJECT_INSIDE) {
    result += result.size() != 4 ? "-OBJ_INS" : "OBJ_INS";
    dptype = (DetectionProblemType)(dptype - MM_OBJECT_INSIDE);
  }
  if(dptype & MM_OBJECT_RIGHT) {
    result += result.size() != 4 ? "-OBJ_RIG" : "OBJ_RIG";
    dptype = (DetectionProblemType)(dptype - MM_OBJECT_RIGHT);
  }
  if(dptype & MM_OBJECT_LEFT) {
    result += result.size() != 4 ? "-OBJ_LEF" : "OBJ_LEF";
    dptype = (DetectionProblemType)(dptype - MM_OBJECT_LEFT);
  }
  if(dptype & MM_DYNAMIC_BOTTOM) {
    result += result.size() != 4 ? "-DYN_BOT" : "DYN_BOT";
    dptype = (DetectionProblemType)(dptype - MM_DYNAMIC_BOTTOM);
  }
  if(dptype & MM_DYNAMIC_TOP) {
    result += result.size() != 4 ? "-DYN_TOP" : "DYN_TOP";
    dptype = (DetectionProblemType)(dptype - MM_DYNAMIC_TOP);
  }
  if(dptype & MM_DYNAMIC_LEFT) {
    result += result.size() != 4 ? "-DYN_LEF" : "DYN_LEF";
    dptype = (DetectionProblemType)(dptype - MM_DYNAMIC_LEFT);
  }
  if(dptype & MM_DYNAMIC_RIGHT) {
    result += result.size() != 4 ? "-DYN_RIG" : "DYN_RIG";
    dptype = (DetectionProblemType)(dptype - MM_DYNAMIC_RIGHT);
  }  
  if(dptype & MM_TOTAL_OCCLUSION) {
    result += result.size() != 4 ? "-TOT_OCC" : "TOT_OCC";
    dptype = (DetectionProblemType)(dptype - MM_TOTAL_OCCLUSION);
  }
  if(dptype & MM_OBJECT_LOST) {
    result += result.size() != 4 ? "-OBJ_LOS" : "OBJ_LOS";
    dptype = (DetectionProblemType)(dptype - MM_OBJECT_LOST);
  }
  if(dptype & MM_PARTIALLY_DETECTED) {
    result += result.size() != 4 ? "-PAR_DET" : "PAR_DET";
    dptype = (DetectionProblemType)(dptype - MM_PARTIALLY_DETECTED);
  }
  if(dptype & MM_PART_OF_BIGGER) {
    result += result.size() != 4 ? "-PAR_BIG" : "PAR_BIG";
    dptype = (DetectionProblemType)(dptype - MM_PART_OF_BIGGER);
  }
  if(dptype != MM_DP_NONE)
    result += result.size() != 4 ? "-OTHER" : "OTHER";

  return result;
  
}

double Blob::computeIntersectRatio(Blob *b1, Blob *b2) {
    return Rectangle<int>::rectangleIntersectRatio(&(b1->bbox),&(b2->bbox));
}

double Blob::computeOverlapRatio(Blob *b1, Blob *b2) {
    if (((b1->bbox.width)*(b1->bbox.height))>((b2->bbox.width)*(b2->bbox.height)))
        return Rectangle<int>::rectangleIntersectRatio(&(b2->bbox),&(b1->bbox));
    else
        return Rectangle<int>::rectangleIntersectRatio(&(b1->bbox),&(b2->bbox));
}

double Blob::computeOverlapRatioStrict(Blob *b1, Blob *b2) {
    return Rectangle<int>::rectangleIntersectRatioStrict(&(b1->bbox),&(b2->bbox));
}

bool Blob::isBlob1OverlapBlob2(Blob *b1, Blob *b2) {
        return Rectangle<int>::rectangleIntersectRatio(&(b2->bbox),&(b1->bbox)) == 0.0 ? false : true;
}

bool Blob::isPixBelongsToBlob(int x, int y) {
    if(x >= bbox.xleft && x <= bbox.xright && y >= bbox.ytop && y <= bbox.ybottom)
        return true;
    return false;
}

void Blob::whichBlobsPixelBelongsTo(std::list<Blob *>& list, std::vector<Blob>& blobs, int x, int y) {
    list.clear();
    if(blobs.empty())
        return;
    std::vector<Blob>::iterator iter, it_end = blobs.end();
    for(iter = blobs.begin(); iter!=it_end; iter++)
        if(iter->isPixBelongsToBlob(x, y))
            list.push_front(&*iter);
}

bool Blob::isBlobOccluded() {
    if (dp_type == MM_CAM_OCCL_TOP ||
        dp_type == MM_CAM_OCCL_BOTTOM ||
        dp_type == MM_CAM_OCCL_LEFT ||
        dp_type == MM_CAM_OCCL_RIGHT ||
        dp_type == MM_CAM_OCCL_TOP_LEFT ||
        dp_type == MM_CAM_OCCL_TOP_RIGHT ||
        dp_type == MM_CAM_OCCL_BOTTOM_LEFT ||
        dp_type == MM_CAM_OCCL_BOTTOM_RIGHT ||
        dp_type == MM_OBJECT_TOP ||
        dp_type == MM_OBJECT_BOTTOM ||
        dp_type == MM_OBJECT_LEFT ||
        dp_type == MM_OBJECT_RIGHT)
        return true;
    return false;
}

bool Blob::isBlobTopOccluded() {
    if (dp_type == MM_CAM_OCCL_TOP ||
        dp_type == MM_CAM_OCCL_TOP_LEFT ||
        dp_type == MM_CAM_OCCL_TOP_RIGHT ||
        dp_type == MM_OBJECT_BOTTOM)
        return true;
    return false;
}
		
bool Blob::isBlobBottomOccluded() {
    if (dp_type == MM_CAM_OCCL_BOTTOM ||
        dp_type == MM_CAM_OCCL_BOTTOM_LEFT ||
        dp_type == MM_CAM_OCCL_BOTTOM_RIGHT ||
        dp_type == MM_OBJECT_TOP)
        return true;
    return false;
}

int Blob::get3DBBox(SceneModel *smodel, double h, double alpha) {

    int eq_relations[4][4] = { {0, 2, 0, 1}    //Given the existence of two types of equations to relate two variables
                              ,{2, 0, 1, 0}   //this matrix stablishes the types of relations between variables.
                              ,{0, 1, 0, 2}   //0: no relation, 1:equation type 1, 2:equation type 2.
                              ,{1, 0, 2, 0} };
    int i,j;
  
    int limits[4][4] = { {1, 0, 0, 0}  //1:If point determine the limit, 0:if not. First index represents a point of the parallelepiped and
                        ,{0, 1, 0, 0} //the second represents the limit (0:Left, 1:Bottom, 2:Right, 3:Top).
                        ,{0, 0, 1, 0}
                        ,{0, 0, 0, 1} };
    int nlimits[4] =     {1, 1, 1, 1}; //Number of limits determined by each variable.

    int varlimrel[4] =     {0, 1, 2, 3}; //Index of 3D variable bounded by the corresponding limit.

    int BlobLimits[4]; //2D blob limits (0:Left, 1:Bottom, 2:Right, 3:Top).

    double V[4][4]; //for each limit(first index), 4 parameters are defined(second index). The third can depend on h, depending on position.

    int position = getPositionRelativeToCamera(smodel);
  
    this->position = position;
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
    int in_height[4]; //Determine if a blob limit is determined by the point in height h (1) or in the base (0);

    double *pp = (double *)SM_CALIB_MATRIX(smodel);
    //Store ordered blob limits
    BlobLimits[0] = bbox.xleft;
    BlobLimits[1] = bbox.ybottom;
    BlobLimits[2] = bbox.xright;
    BlobLimits[3] = bbox.ytop;

    for(i=0;i<4;i++)
        for(j=0;j<4;j++)
            V[i][j] = pp[(i%2)*4 + j] - BlobLimits[i]*pp[8 + j]; //Now V3,V6,V9,V12 will be of type Vn = Vn1*h + Vn2

    //Determine which limits will intersect with projections of points with height h.
    setInHeightGivenPosition(position, in_height);

    if( compute3DBBoxNormalCase(smodel, h, alpha, V, limits, eq_relations, in_height) )
        return 1;

    BLOB_3DBBOX(this)->getLimits(limits, nlimits, varlimrel);

    return compute3DBBoxSpecialCases(smodel, h, alpha, V, limits, nlimits, eq_relations, in_height);
  
}

int Blob::compute3DBBoxNormalCase(SceneModel *smodel, double h, double alpha, double V[4][4], int limits[4][4], int eq_relations[4][4], int in_height[4]) {

    int i,j;
    double X[4],Y[4];
    double sina = sin(alpha);
    double cosa = cos(alpha);
    int l[4];
    for(i=0; i<4; i++)
        for(j=0; j<4; j++)
            if(limits[i][j] == 1)
                l[j] = i;
 
    double
        Vl0 =  V[0][0],
        Vl1 =  V[0][1],
        Vl2 = (V[0][2]*in_height[0]*h + V[0][3]),
        Vb0 =  V[1][0],
        Vb1 =  V[1][1],
        Vb2 = (V[1][2]*in_height[1]*h + V[1][3]),
        Vr0 =  V[2][0],
        Vr1 =  V[2][1],
        Vr2 = (V[2][2]*in_height[2]*h + V[2][3]),
        Vt0 =  V[3][0],
        Vt1 =  V[3][1],
        Vt2 = (V[3][2]*in_height[3]*h + V[3][3]);

    if(eq_relations[l[1]][l[2]] == 1) {
        X[l[1]] = (
	    Vb1*(((Vr0*Vt0 - Vr1*Vt1)*sina - (Vr0*Vt1 + Vr1*Vt0)*cosa)*cosa + Vr1*Vt0)*Vl2
	    + ((2*Vl1*Vt0*Vr1*cosa - (Vl1*(Vr0*Vt0 - Vr1*Vt1) - Vl0*(Vr0*Vt1 - Vr1*Vt0))*sina)*cosa - Vl1*Vr1*Vt0)*Vb2
	    + Vb1*(((Vl0*Vt0 - Vl1*Vt1)*sina - (Vl0*Vt1 + Vl1*Vt0)*cosa)*cosa + Vl0*Vt1)*Vr2
	    - Vb1*(((Vl0*Vr0 - Vl1*Vr1)*sina - (Vl0*Vr1 + Vl1*Vr0)*cosa)*cosa + Vl0*Vr1)*Vt2
	   )/( 
              (2*(Vl0*Vb1*Vr0*Vt1 - Vl1*Vb0*Vr1*Vt0)*cosa + (  (Vb0*Vr1 - Vb1*Vr0)*(Vl0*Vt0 - Vl1*Vt1) - (Vb0*Vr0 - Vb1*Vr1)*(Vl0*Vt1 - Vl1*Vt0) )*sina)*cosa
            - (Vl0*Vb1*Vr0*Vt1 - Vl1*Vt0*Vb0*Vr1)
                );
        Y[l[1]] = - X[l[1]]  *Vb0/Vb1 - Vb2/Vb1;

        X[l[2]] = (Vr1*X[l[1]]*sina - (Vr2 + Vr1*Y[l[1]])*cosa)/(Vr0*cosa + Vr1*sina);
        Y[l[2]] = - X[l[2]]*Vr0/Vr1 - Vr2/Vr1;

        X[l[0]] = (Vl1*X[l[1]]*cosa + (Vl2 + Vl1*Y[l[1]])*sina)/(Vl1*cosa - Vl0*sina);
        Y[l[0]] = - X[l[0]]*Vl0/Vl1 - Vl2/Vl1;

        X[l[3]] = (Vt1*X[l[2]]*cosa + (Vt2 + Vt1*Y[l[2]])*sina)/(Vt1*cosa - Vt0*sina);
        Y[l[3]] = - X[l[3]]*Vt0/Vt1 - Vt2/Vt1;

    } else { //Relation of type 2
        X[l[1]] = (
	    + Vb1*(((Vr0*Vt0 - Vt1*Vr1)*sina - (Vr0*Vt1 + Vr1*Vt0)*cosa)*cosa + Vr0*Vt1)*Vl2
	    + ((2*Vl1*Vt0*Vr1*cosa - (Vl1*(Vr0*Vt0 - Vr1*Vt1) - Vl0*(Vr0*Vt1 - Vr1*Vt0))*sina)*cosa - Vl1*Vr1*Vt0)*Vb2
	    + Vb1*(((Vl0*Vt0 - Vt1*Vl1)*sina - (Vl0*Vt1 + Vl1*Vt0)*cosa)*cosa + Vl1*Vt0)*Vr2
	    - Vb1*(((Vl0*Vr0 - Vl1*Vr1)*sina - (Vl0*Vr1 + Vl1*Vr0)*cosa)*cosa + Vl1*Vr0)*Vt2
	   )/(
              (2*(Vl0*Vb1*Vr0*Vt1 - Vl1*Vb0*Vr1*Vt0)*cosa - ((Vl0*Vb0 - Vb1*Vl1)*(Vr0*Vt1 - Vr1*Vt0) + (Vl0*Vb1 - Vl1*Vb0)*(Vr0*Vt0 - Vr1*Vt1))*sina)*cosa
            - (Vl0*Vb1*Vr0*Vt1 - Vl1*Vb0*Vr1*Vt0)
	   );
        Y[l[1]]   = - X[l[1]]  *Vb0/Vb1 - Vb2/Vb1;

        X[l[2]] = (Vr1*X[l[1]]*cosa + (Vr2 + Vr1*Y[l[1]])*sina)/(Vr1*cosa - Vr0*sina);
        Y[l[2]] = - X[l[2]]*Vr0/Vr1 - Vr2/Vr1;

        X[l[0]] = (Vl1*X[l[1]]*sina - (Vl2 + Vl1*Y[l[1]])*cosa)/(Vl0*cosa + Vl1*sina);
        Y[l[0]] = - X[l[0]]*Vl0/Vl1 - Vl2/Vl1;

        X[l[3]] = (Vt1*X[l[2]]*sina - (Vt2 + Vt1*Y[l[2]])*cosa)/(Vt0*cosa + Vt1*sina);
        Y[l[3]] = - X[l[3]]*Vt0/Vt1 - Vt2/Vt1;
    }

    return set3DBBox(smodel, (double *)X, (double *)Y, h);
}

bool Blob::set3DBBox(SceneModel *smodel, double *X, double *Y, double h){
    int i;
    double x2d = 0.0, y2d = 0.0;
    int accomplished_constraints = 0;
    int L = bbox.xleft - 1,
        B = bbox.ybottom + 1,
        R = bbox.xright + 1,
        T = bbox.ytop - 1;

    _3Dbbox.h = h;
    for(i=0; i<4; i++) {
        BLOB_3DBBOX_X_i(this, i) = X[i];
        BLOB_3DBBOX_Y_i(this, i) = Y[i];
        SceneModel::worldToImgCoords(SM_CALIB_MATRIX(smodel), X[i], Y[i], 0, &x2d, &y2d);
        BLOB_3DBBOX_X2D_BASE_i(this, i) = x2d;
        BLOB_3DBBOX_Y2D_BASE_i(this, i) = y2d;
        if(x2d >= L && x2d <= R && y2d >= T && y2d <= B)
            accomplished_constraints++;
        SceneModel::worldToImgCoords(SM_CALIB_MATRIX(smodel), X[i], Y[i], h, &(x2d), &(y2d));
        BLOB_3DBBOX_X2D_H_i(this, i) = x2d;
        BLOB_3DBBOX_Y2D_H_i(this, i) = y2d;
        if((x2d >= L) && (x2d <= R) && (y2d >= T) && (y2d <= B))
            accomplished_constraints++;
  }

  if(accomplished_constraints == 8)
        return true;

  return false;
}

int Blob::compute3DBBoxSpecialCases(SceneModel *smodel, double h, double alpha, double V[4][4], int limits[4][4], int nlimits[4], int eq_relations[4][4], int in_height[4]) {
    int i, j = -1, k, lim_count = 0;

    for(i=0; i<4; i++) {
        if(j < 0 && nlimits[i] == 2)
            j=i;
        if(nlimits[i]>0)
            lim_count++;
    }

    if(lim_count == 4) //If each point is on one limit
        return compute3DBBoxNormalCase(smodel, h, alpha, V, limits, eq_relations, in_height);

    //If we have arrived here, this is a special case of a point 'j' with two limits
    double X[4],Y[4];
    double sina = sin(alpha);
    double cosa = cos(alpha);
    int a, b, c, d;
    int Case = 10000;

    //Search for the two limits
    if(j < 0) { //Degenerate case where point is in three limits, returns normal blob with default values
        int limits_aux[4][4] = { {1, 0, 0, 0}
                                ,{0, 1, 0, 0}
                                ,{0, 0, 1, 0}
                                ,{0, 0, 0, 1} };
        return compute3DBBoxNormalCase(smodel, h, alpha, V, limits_aux, eq_relations, in_height);
    }

    a = -1;
    for(i=0; i<4; i++) {
        if(a < 0 && limits[j][i])
            a = i;
        else if(limits[j][i]) {
            b = i;
            break;
        }
    }

    if (nlimits[(j+1) % 4] == 1) { //If other points belongs to one limit (following to the right)
        for(i=0; i<4; i++)
            if(limits[(j+1) % 4][i]) {
                c = i;
                break;
            }
        if(nlimits[(j+2) % 4] == 1) { //The last limit is on j+2
            Case = 0;
            for(i=0; i<4; i++)
                if(limits[(j+2) % 4][i]) {
                    d = i;
                    break;
                }
        } else { //point j+3 has 1 limit
            Case = 1;
            for(i=0; i<4; i++)
            if(limits[(j+3) % 4][i]) {
                d = i;
                break;
            }
        }
    } else if(nlimits[(j+3) % 4] == 1) { //If one point has one limit the case is clear: (j+3) and (j+2) are in limits
        Case = 2;
        for(i=0; i<4; i++)
            if(limits[(j+3) % 4][i]) {
                c = i;
                break;
            }
        for(i=0; i<4; i++)
            if(limits[(j+2) % 4][i]) {
                d = i;
                break;
            }
    } else if(nlimits[(j+2) % 4] == 2) { //case double 2 limits
        Case = 3;
        for(i=j+1; i<4; i++) //Search the next point with two limits
            if(nlimits[i]==2) {
                k=i;
                break;
            }
        c=-1;
        for(i=0; i<4; i++) {
            if(c<0 && limits[k][i])
                c = i;
            else if(limits[k][i]) {
                d = i;
                break;
            }
        }
    } else { //Degenerate case, returns normal case with default configuration
        int limits_aux[4][4] = {  {1, 0, 0, 0}
                                 ,{0, 1, 0, 0}
                                 ,{0, 0, 1, 0}
                                 ,{0, 0, 0, 1} };
        return compute3DBBoxNormalCase(smodel, h, alpha, V, limits_aux, eq_relations, in_height);
    }

    double
        Va0 =  V[a][0],
        Va1 =  V[a][1],
        Va2 = (V[a][2]*in_height[a]*h + V[a][3]),
        Vb0 =  V[b][0],
        Vb1 =  V[b][1],
        Vb2 = (V[b][2]*in_height[b]*h + V[b][3]),
        Vc0 =  V[c][0],
        Vc1 =  V[c][1],
        Vc2 = (V[c][2]*in_height[c]*h + V[c][3]),
        Vd0 =  V[d][0],
        Vd1 =  V[d][1],
        Vd2 = (V[d][2]*in_height[d]*h + V[d][3]);

    X[j] =  (Va1*Vb2 - Va2*Vb1)/(Va0*Vb1 - Va1*Vb0);
    Y[j] = -(Va0*Vb2 - Va2*Vb0)/(Va0*Vb1 - Va1*Vb0);
  
    if (Case < 2) {
        if (eq_relations[j][(j+1)%4] == 1) {
            X[(j+1)%4] = (Vc1*X[j]*sina - (Vc2 + Vc1*Y[j])*cosa)/(Vc0*cosa + Vc1*sina);
        } else {
            X[(j+1)%4] = (Vc1*X[j]*cosa + (Vc2 + Vc1*Y[j])*sina)/(Vc1*cosa - Vc0*sina);
        }
        Y[(j+1)%4] = - X[(j+1)%4]*Vc0/Vc1 - Vc2/Vc1;

        if (Case == 1) {
            if (eq_relations[j][(j+3)%4] == 1) {
                X[(j+3)%4] = (Vd1*X[j]*sina - (Vd2 + Vd1*Y[j])*cosa)/(Vd0*cosa + Vd1*sina);
            } else {
                X[(j+3)%4] = (Vd1*X[j]*cosa + (Vd2 + Vd1*Y[j])*sina)/(Vd1*cosa - Vd0*sina);
            }
            Y[(j+3)%4] = - X[(j+3)%4]*Vd0/Vd1  - Vd2/Vd1;

            if (eq_relations[(j+1)%4][(j+2)%4] == 1) {
                X[(j+2)%4] = X[(j+1)%4] + ( (X[(j+3)%4] - X[(j+1)%4])*cosa + (Y[(j+3)%4] - Y[(j+1)%4])*sina )*cosa;
                Y[(j+2)%4] = Y[(j+3)%4] - ( (Y[(j+3)%4] - Y[(j+1)%4])*cosa - (X[(j+3)%4] - X[(j+1)%4])*sina )*cosa;
            } else {
                X[(j+2)%4] = X[(j+3)%4] + ( (X[(j+1)%4] - X[(j+3)%4])*cosa + (Y[(j+1)%4] - Y[(j+3)%4])*sina )*cosa;
                Y[(j+2)%4] = Y[(j+1)%4] - ( (Y[(j+1)%4] - Y[(j+3)%4])*cosa - (X[(j+1)%4] - X[(j+3)%4])*sina )*cosa;
            }
        } else {// Case 0: JUST ONE ON THE SIDE
            if (eq_relations[(j+1)%4][(j+2)%4] == 1) {
                X[(j+2)%4] = (Vd1*X[(j+1)%4]*sina - (Vd2 + Vd1*Y[(j+1)%4])*cosa)/(Vd0*cosa + Vd1*sina);
            } else {
                X[(j+2)%4] = (Vd1*X[(j+1)%4]*cosa + (Vd2 + Vd1*Y[(j+1)%4])*sina)/(Vd1*cosa - Vd0*sina);
            }
            Y[(j+2)%4] = - X[(j+2)%4]*Vd0/Vd1  - Vd2/Vd1;

            if (eq_relations[(j+3)%4][j] == 1) {
                X[(j+3)%4] = X[j]   + ( (X[(j+2)%4] - X[j])*cosa + (Y[(j+2)%4] - Y[j])*sina )*cosa;
                Y[(j+3)%4] = Y[(j+2)%4] - ( (Y[(j+2)%4] - Y[j])*cosa - (X[(j+2)%4] - X[j])*sina )*cosa;
            } else {
                X[(j+3)%4] = X[(j+2)%4] + ( (X[j] - X[(j+2)%4])*cosa + (Y[j] - Y[(j+2)%4])*sina )*cosa;
                Y[(j+3)%4] = Y[j]   - ( (Y[j] - Y[(j+2)%4])*cosa - (X[j] - X[(j+2)%4])*sina )*cosa;
            }
        }
    } else if (Case == 2) {
        if (eq_relations[j][(j+3)%4] == 1) {
            X[(j+3)%4] = (Vc1*X[j]*sina - (Vc2 + Vc1*Y[j])*cosa)/(Vc0*cosa + Vc1*sina);
        } else {
            X[(j+3)%4] = (Vc1*X[j]*cosa + (Vc2 + Vc1*Y[j])*sina)/(Vc1*cosa - Vc0*sina);
        }
        Y[(j+3)%4] = - X[(j+3)%4]*Vc0/Vc1 - Vc2/Vc1;

        if (eq_relations[(j+3)%4][(j+2)%4] == 1) {
            X[(j+2)%4] = (Vd1*X[(j+3)%4]*sina - (Vd2 + Vd1*Y[(j+3)%4])*cosa)/(Vd0*cosa + Vd1*sina);
        } else {
            X[(j+2)%4] = (Vd1*X[(j+3)%4]*cosa + (Vd2 + Vd1*Y[(j+3)%4])*sina)/(Vd1*cosa - Vd0*sina);
        }
        Y[(j+2)%4] = - X[(j+2)%4]*Vd0/Vd1  - Vd2/Vd1;

        if (eq_relations[(j+1)%4][j] == 1) {
            X[(j+1)%4] = X[j]   + ( (X[(j+2)%4] - X[j])*cosa + (Y[(j+2)%4] - Y[j])*sina )*cosa;
            Y[(j+1)%4] = Y[(j+2)%4] - ( (Y[(j+2)%4] - Y[j])*cosa - (X[(j+2)%4] - X[j])*sina )*cosa;
        } else {
            X[(j+1)%4] = X[(j+2)%4] + ( (X[j] - X[(j+2)%4])*cosa + (Y[j] - Y[(j+2)%4])*sina )*cosa;
            Y[(j+1)%4] = Y[j]   - ( (Y[j] - Y[(j+2)%4])*cosa - (X[j] - X[(j+2)%4])*sina )*cosa;
        }
    } else { //Case 3: TWO AGAINST TWO

        X[(j+2)%4] =  (Vc1*Vd2 - Vc2*Vd1)/(Vc0*Vd1 - Vc1*Vd0);
        Y[(j+2)%4] = -(Vc0*Vd2 - Vc2*Vd0)/(Vc0*Vd1 - Vc1*Vd0);

        if (eq_relations[(j+1)%4][j] == 1) {
            X[(j+1)%4] = X[j]   + ( (X[(j+2)%4] - X[j])*cosa + (Y[(j+2)%4] - Y[j])*sina )*cosa;
            Y[(j+1)%4] = Y[(j+2)%4] - ( (Y[(j+2)%4] - Y[j])*cosa - (X[(j+2)%4] - X[j])*sina )*cosa;
        } else {
            X[(j+1)%4] = X[(j+2)%4] + ( (X[j] - X[(j+2)%4])*cosa + (Y[j] - Y[(j+2)%4])*sina )*cosa;
            Y[(j+1)%4] = Y[j]   - ( (Y[j] - Y[(j+2)%4])*cosa - (X[j] - X[(j+2)%4])*sina )*cosa;
        }

        if (eq_relations[(j+3)%4][j] == 1) {
            X[(j+3)%4] = X[j]   + ( (X[(j+2)%4] - X[j])*cosa + (Y[(j+2)%4] - Y[j])*sina )*cosa;
            Y[(j+3)%4] = Y[(j+2)%4] - ( (Y[(j+2)%4] - Y[j])*cosa - (X[(j+2)%4] - X[j])*sina )*cosa;
        } else {
            X[(j+3)%4] = X[(j+2)%4] + ( (X[j] - X[(j+2)%4])*cosa + (Y[j] - Y[(j+2)%4])*sina )*cosa;
            Y[(j+3)%4] = Y[j]   - ( (Y[j] - Y[(j+2)%4])*cosa - (X[j] - X[(j+2)%4])*sina )*cosa;
        }
    }

    return set3DBBox(smodel, (double *)X, (double *)Y, h);
}

void Blob::setInHeightGivenPosition(int position, int *in_height) {
#if DEBUG_OUTPUT
    printf("Position:\t");
#endif
    switch(position){ //Determine activation of h dependant constant.
        case 0:
#if DEBUG_OUTPUT
            printf("Left-Front.\n");
#endif
            in_height[0]=in_height[3]=1; in_height[1]=in_height[2]=0; break;
        case 1:
#if DEBUG_OUTPUT
            printf("Middle-Front.\n");
#endif
            in_height[0]=in_height[2]=in_height[3]=1; in_height[1]=0; break;
        case 2:
#if DEBUG_OUTPUT
            printf("Right-Front.\n");
#endif
            in_height[2]=in_height[3]=1; in_height[0]=in_height[1]=0; break;
        case 3:
#if DEBUG_OUTPUT
            printf("Left-Center.\n");
#endif
            in_height[0]=in_height[1]=in_height[3]=1; in_height[2]=0; break;
        case 4:
#if DEBUG_OUTPUT
            printf("Middle-Center.\n");
#endif
            in_height[0]=in_height[1]=in_height[2]=in_height[3]=1; break;
        case 5:
#if DEBUG_OUTPUT
            printf("Right-Center.\n");
#endif
            in_height[1]=in_height[2]=in_height[3]=1; in_height[0]=0; break;
        case 6:
#if DEBUG_OUTPUT 
            printf("Left-Back.\n");
#endif
            in_height[0]=in_height[1]=1; in_height[2]=in_height[3]=0; break;
        case 7:
#if DEBUG_OUTPUT
            printf("Middle-Back.\n");
#endif
            in_height[0]=in_height[1]=in_height[2]=1; in_height[3]=0; break;
        case 8:
#if DEBUG_OUTPUT
            printf("Right-Front.\n");
#endif
            in_height[1]=in_height[2]=1; in_height[0]=in_height[3]=0; break;
        default:
#if DEBUG_OUTPUT
            printf("\nWarning: Position %d doesn't exist. Assuming position 1.\n", position);
#endif
            in_height[0]=in_height[2]=in_height[3]=1; in_height[1]=0;
    }
}

//New implementation independent from the focal point
int Blob::getPositionRelativeToCamera(SceneModel *smodel) {
    return bbox.getPositionRelativeToCamera(smodel);
}


int Blob::getPositionRelativeToCameraOld(SceneModel *smodel) {
    return bbox.getPositionRelativeToCameraOld(smodel);
}

void Blob::setReal3DPosition() {
    _3Dbbox.setReal3DPosition(&position3D);
}

void Blob::setEstimated3DPosition(SceneModel* smodel) {
    bbox.setEstimated3DPosition(smodel, position3D, position);
}

void Blob::setPositionAtCenterBottom(SceneModel* smodel) {
    bbox.setPositionAtCenterBottom(smodel, position3D);
}

bool Blob::isBlob1InsideBlob2(Blob *b1, Blob *b2) {
    return Rectangle<int>::isRect1InsideRect2(&b1->bbox, &b2->bbox);
}

#define BLOB_PIXEL_ERROR 0.5

bool Blob::same2DBlob(Blob *b1, Blob *b2) {
  double
    bottom = BLOB_YBOTTOM(b1) - BLOB_YBOTTOM(b2),
    top    = BLOB_YTOP(b1)    - BLOB_YTOP(b2),
    left   = BLOB_XLEFT(b1)   - BLOB_XLEFT(b2),
    right  = BLOB_XRIGHT(b1) - BLOB_XRIGHT(b2);

  if( fabs(bottom) <= BLOB_PIXEL_ERROR && fabs(top) <= BLOB_PIXEL_ERROR && fabs(left) <= BLOB_PIXEL_ERROR && fabs(right) <= BLOB_PIXEL_ERROR )
    return true;

  return false;
}

void Blob::set2Dand3Dbboxes(SceneModel *smodel, double *x, double *y, Rectangle<int> *bb2D, Parallelpiped *bb3D, double h) {
    int i;
    double x2d, y2d;
    double L = BL_INFINITY, T = BL_INFINITY, B = 0, R = 0;
  
    PARALL_H(bb3D) = h;
  
    for(i = 0; i < 4; i++) {
        PARALL_X_i(bb3D, i) = x[i];
        PARALL_Y_i(bb3D, i) = y[i];
        SceneModel::worldToImgCoords(SM_CALIB_MATRIX(smodel), x[i], y[i], 0.0, &x2d, &y2d);
        PARALL_X2D_BASE_i(bb3D, i) = x2d;
        PARALL_Y2D_BASE_i(bb3D, i) = y2d;
        if(x2d < L)
            L = x2d;
        if(x2d > R)
            R = x2d;
        if(y2d < T)
            T = y2d;
        if(y2d > B)
            B = y2d;
        SceneModel::worldToImgCoords(SM_CALIB_MATRIX(smodel), x[i], y[i], h, &x2d, &y2d);
        PARALL_X2D_H_i(bb3D, i) = x2d;
        PARALL_Y2D_H_i(bb3D, i) = y2d;
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


double Blob::horizontalDistanceToPoint(double X2Dfpoint) {
    double blobX2D = bbox.xleft > X2Dfpoint ? bbox.xleft : (bbox.xright < X2Dfpoint ? bbox.xright : X2Dfpoint);
    return fabs(blobX2D - X2Dfpoint);
}

double Blob::verticalDistanceToPoint(double Y2Dfpoint) {
    double blobY2D = bbox.ytop > Y2Dfpoint ? bbox.ytop : (bbox.ybottom < Y2Dfpoint ? bbox.ybottom : Y2Dfpoint);
    return fabs(blobY2D - Y2Dfpoint);
}

void Blob::orderByProximityToPoint(std::vector<Blob>& blobs, double x, double y) {
    int i, j, blobsNumber = blobs.size(), top, bottom;

    if(blobsNumber == 0)
        return;

    double *dXvalues = new double[blobsNumber], *dYvalues = new double[blobsNumber];

    for(i=0; i<blobsNumber; i++) {
        dXvalues[i] = blobs[i].horizontalDistanceToPoint(x);
        dYvalues[i] = blobs[i].verticalDistanceToPoint(y);
    }

    orderBlobsAscending(dYvalues, blobs, 0, blobsNumber - 1);

    for(i=0; i<blobsNumber; i++) {
        bottom = top = i;
        for(j = i+1; j<blobsNumber; j++)
        if(dYvalues[top] == dYvalues[j])
            bottom++;
        else
            break;
        if(bottom > top)
            orderBlobsAscending(dXvalues, blobs, top, bottom);
        i = j - 1;
    }

    delete[] dXvalues;
    delete[] dYvalues;
}

//Quicksort
void Blob::orderBlobsAscending(double *values, std::vector<Blob>& blobs, int top, int bottom) {
    int middle;
    if (top < bottom) {
      middle = getMiddle(values, blobs, top, bottom);
      orderBlobsAscending(values, blobs, top, middle);   // sort top partition
      orderBlobsAscending(values, blobs, middle+1, bottom);    // sort bottom partition
    }
}

  int Blob::getMiddle(double *values, std::vector<Blob>& blobs, int top, int bottom) {
    int  i = top - 1, j = bottom + 1;
    double x = values[top], temp;
    Blob blobTemp;

    do {
      do {
        j --;
      }while (x < values[j]);

      do {
        i++;
      } while (x > values[i]);

      if (i < j) {
        temp = values[i];    // switch elements at positions i and j
        blobTemp = blobs[i];

        values[i] = values[j];
        blobs[i] = blobs[j];

        values[j] = temp;
        blobs[j] = blobTemp;
      }
    }while (i < j);

    return j;           // returns middle index
}

Shape3DData *Blob::getBestS3D(bool &lastUnknown, ObjectType best_type, double classifThreshold) {

    if(gP < classifThreshold) {
        lastUnknown = true;
        return NULL;
    }

    Shape3DData *bests3d, *occ_s3d, *normal_s3d;

    normal_s3d = &(*normal_data)[best_type];
    if(occ_data)
        occ_s3d = &(*occ_data)[best_type];

    if(S3D_P(normal_s3d) < classifThreshold) {
        if(occ_data) {
            if(S3D_P(occ_s3d) < classifThreshold) {
                bests3d = NULL;
                lastUnknown = true;
            } else {
                bests3d = occ_s3d;
                lastUnknown = false;
            }
        } else {
            bests3d = NULL;
            lastUnknown = true;
        }
    } else {
        if(occ_data) {
            if(S3D_P(occ_s3d) > S3D_P(normal_s3d))
                bests3d = occ_s3d;
            else
                bests3d = normal_s3d;
        } else
            bests3d = normal_s3d;
        lastUnknown = false;
    }

    return bests3d;
}



densityData::densityData() { memset(this, 0, sizeof(densityData)); }

densityData::~densityData() {}

Shape3DData::Shape3DData() {
    memset(this, 0, sizeof(Shape3DData));
    subtypesList = NULL;
}

Shape3DData::~Shape3DData() {
    if(subtypesList != NULL)
        delete subtypesList;
    subtypesList = NULL;
}

Shape3DData& Shape3DData::operator=(const Shape3DData& s) {
    memcpy(this, &s, sizeof(Shape3DData));
    if(s.subtypesList != NULL) {
        if(subtypesList != NULL)
            delete subtypesList;
        subtypesList = new std::map<ObjectSubtype, Shape3DData>();
        *subtypesList = *(s.subtypesList);
    }
    return *this;
}


Shape3DData *Shape3DData::copy() {
    Shape3DData *Copy = new Shape3DData();
    memcpy(Copy, this, sizeof(Shape3DData));
    if(subtypesList != NULL) {
        Copy->subtypesList = new std::map<ObjectSubtype, Shape3DData>();
        *(Copy->subtypesList) = *subtypesList;
    } else
        Copy->subtypesList = NULL;
    return Copy;
}


void Shape3DData::copy(Shape3DData *copy_here) {
    if(copy_here == NULL)
        return;
    if(copy_here->subtypesList != NULL)
        delete copy_here->subtypesList;
    memcpy(copy_here, this, sizeof(Shape3DData));
    if(subtypesList != NULL) {
        copy_here->subtypesList = new std::map<ObjectSubtype, Shape3DData>();
        *(copy_here->subtypesList) = *subtypesList;
    } else
        copy_here->subtypesList = NULL;
}

//Set null 3D information for s3d, just copy 2D bbox from blob
void Shape3DData::setNull(Blob *blob) {
    
    ObjectType ttype     = type;
    ObjectSubtype stype = subtype;
    std::map<ObjectSubtype, Shape3DData> *slist = subtypesList;

    memset(this, 0, sizeof(Shape3DData));

    S3D_TYPE(this) = ttype;
    S3D_SUBTYPE(this) = stype;
    subtypesList = slist;
    if(blob != NULL) {
        memcpy(S3D_BBOX(this), BLOB_BBOX(blob), sizeof(Rectangle<int>));
        this->maxDistanceFactor = blob->maxDistanceFactor;
    }
}

//Set null 3D information for s3d, just copy 2D bbox from blob
void Shape3DData::setNull(ObjectType type, ObjectSubtype subtype, DetectionProblemType occtype, Rectangle<int> *bbox) {

    Rectangle<int> aux;
    std::map<ObjectSubtype, Shape3DData> *slist = subtypesList;
    double maxDistanceFactor = this->maxDistanceFactor;
    //This copy is made to prevent erasing the information with next memset (if rect is part of prd)
    memcpy(&aux, bbox, sizeof(Rectangle<int>));

    memset(this, 0, sizeof(Shape3DData));

    this->type = type;
    this->subtype = subtype;
    this->dp_type = occtype;
    this->maxDistanceFactor = maxDistanceFactor;
    memcpy(&this->bbox, &aux, sizeof(Rectangle<int>));

    subtypesList = slist;
}


void Blob::copyBlobData(Blob *blob1, Blob *blob2) {

    if(blob1 != NULL && blob2 != NULL) {
        int ident = BLOB_IDENT(blob2);
        if(BLOB_NORMAL_3DDATA(blob2) != NULL)
            delete BLOB_NORMAL_3DDATA(blob2);
        if(BLOB_OCC_3DDATA(blob2) != NULL)
            delete BLOB_OCC_3DDATA(blob2);
        if (BLOB_MOVING_PIXELS(blob2) != NULL)
          delete BLOB_MOVING_PIXELS(blob2);

        memcpy(blob2, blob1, sizeof(Blob));

        BLOB_IDENT(blob2) = ident;
        BLOB_NORMAL_3DDATA(blob2) = new std::map<ObjectType, Shape3DData>();
        BLOB_OCC_3DDATA(blob2) = new std::map<ObjectType, Shape3DData>();
        *BLOB_NORMAL_3DDATA(blob2) = *BLOB_NORMAL_3DDATA(blob1);
        *BLOB_OCC_3DDATA(blob2) = *BLOB_OCC_3DDATA(blob1);
        BLOB_MOVING_PIXELS(blob2) = NULL;
    }
}

void Shape3DData::copyBlobToShape3DData(Blob *blob) {
    S3D_TYPE(this)   = BLOB_TYPE(blob);
    S3D_SUBTYPE(this)= BLOB_SUBTYPE(blob);

    S3D_W(this)      = BLOB_3D_WIDTH(blob);
    S3D_H(this)      = BLOB_3D_HEIGHT(blob);
    S3D_L(this)      = BLOB_3D_LENGTH(blob);
    S3D_PW(this)     = BLOB_PW(blob);
    S3D_PH(this)     = BLOB_PH(blob);
    S3D_PL(this)     = BLOB_PL(blob);
    S3D_RW(this)     = BLOB_RW(blob);
    S3D_RH(this)     = BLOB_RH(blob);
    S3D_RL(this)     = BLOB_RL(blob);
    S3D_R(this)      = BLOB_R(blob);
    S3D_P(this)      = BLOB_P(blob);
    S3D_ALPHA(this)  = BLOB_ALPHA(blob);
    this->maxDistanceFactor = blob->maxDistanceFactor;

    memcpy(S3D_BBOX(this), BLOB_BBOX(blob), sizeof(Rectangle<int>));
    memcpy(S3D_3DBBOX(this), BLOB_3DBBOX(blob), sizeof(Parallelpiped));
    memcpy(S3D_DDATA(this), BLOB_DDATA(blob), sizeof(densityData));

}

void Shape3DData::copyNoLists(Shape3DData *copy_here) {
    if(copy_here == NULL)
        return;
    memcpy(copy_here, this, sizeof(Shape3DData));
}

void Shape3DData::copyShape3DDataOnlyInfo(Shape3DData *copy_here) {
    if(copy_here == NULL)
        return;

    std::map<ObjectSubtype, Shape3DData> *subtypes = S3D_SUBTYPES_LIST(copy_here);
    memcpy(copy_here, this, sizeof(Shape3DData));
    S3D_SUBTYPES_LIST(copy_here) = subtypes;
}

void Shape3DData::copyShape3DDataToBlob(Blob *blob) {
    BLOB_TYPE(blob)      = S3D_TYPE(this);
    BLOB_SUBTYPE(blob)   = S3D_SUBTYPE(this);
    BLOB_3D_WIDTH(blob)  = S3D_W(this);
    BLOB_3D_HEIGHT(blob) = S3D_H(this);
    BLOB_3D_LENGTH(blob) = S3D_L(this);
    BLOB_PW(blob)        = S3D_PW(this);
    BLOB_PH(blob)        = S3D_PH(this);
    BLOB_PL(blob)        = S3D_PL(this);
    BLOB_RW(blob)        = S3D_RW(this);
    BLOB_RH(blob)        = S3D_RH(this);
    BLOB_RL(blob)        = S3D_RL(this);
    BLOB_R(blob)         = S3D_R(this);
    BLOB_P(blob)         = S3D_P(this);
    BLOB_ALPHA(blob)     = S3D_ALPHA(this);
    blob->maxDistanceFactor = this->maxDistanceFactor;

    memcpy(BLOB_BBOX(blob), S3D_BBOX(this), sizeof(Rectangle<int>));
    memcpy(BLOB_3DBBOX(blob), S3D_3DBBOX(this), sizeof(Parallelpiped));
    memcpy(BLOB_DDATA(blob), S3D_DDATA(this), sizeof(densityData));
    memcpy(BLOB_3D_POSITION(blob), S3D_3D_POSITION(this), sizeof(point3D<double>));
}

std::map<ObjectType, Shape3DData>* Shape3DData::copyList(std::map<ObjectType, Shape3DData>* list) {
    if(list == NULL)
        return NULL;
    std::map<ObjectType, Shape3DData> *new_list = new std::map<ObjectType, Shape3DData>();
    *new_list = *list;

    return new_list;
}

//Rendering Functions
std::ostream& operator<<(std::ostream& out, const Blob& bb) {
    const Blob *current_blob = &bb;
    out << "\t\t\t\tBlob (ID,type,P,w,l,h,alpha) : (" << BLOB_IDENT(current_blob) << ", ";
    out << Blob::getNameFromType(BLOB_TYPE(current_blob)) << ", " << BLOB_P(current_blob) << ", ";
    out << BLOB_3D_WIDTH(current_blob) << ", " << BLOB_3D_LENGTH(current_blob) << ", "<<BLOB_3D_HEIGHT(current_blob);
    out << ", "<<BLOB_ALPHA(current_blob)<<")" << std::endl;
    out << "\t\t\t\t\t Detection Problem: " << Blob::getDPNameFromTypeDetailed(S3D_DP_TYPE(current_blob)) << std::endl;
    out << "\t\t\t\t\t (x,y,z)           : (" << BLOB_3D_X(current_blob);
    out << ", " <<BLOB_3D_Y(current_blob) << ", " << BLOB_3D_Z(current_blob) << ")" << std::endl;
    out << "\t\t\t\t\t" << " (W,L) : (" << BLOB_WIDTH(current_blob) << ", " << BLOB_HEIGHT(current_blob) << ")" << std::endl;
    out << "\t\t\t\t\t" << " (L,R,T,B) : (" << BLOB_XLEFT(current_blob) << ", "
                                            << BLOB_XRIGHT(current_blob) << ", "
                                            << BLOB_YTOP(current_blob) << ", "
                                            << BLOB_YBOTTOM(current_blob) << ")" << std::endl;
    out << "\t\t\t\t\t" << " maxDistanceFactor : " <<  current_blob->maxDistanceFactor << std::endl;
    if(current_blob->normal_data == NULL)
        out << "\t\t\t\t\t" << " Normal Data : NULL" << std::endl;
    else {
        out << "\t\t\t\t\t" << " Normal Data :" << std::endl;
        std::map<ObjectType, Shape3DData>::iterator it, it_end = current_blob->normal_data->end();
        for(it = current_blob->normal_data->begin(); it != it_end; it++) {
            out << "\t\t\t\t\t\t" << " Type :" << Blob::getNameFromType(it->first) << std::endl;
            out << it->second << std::endl;
        }
    }

    if(current_blob->occ_data == NULL)
        out << "\t\t\t\t\t" << " Occlusion Data : NULL" << std::endl;
    else {
        out << "\t\t\t\t\t" << " Occlusion Data :" << std::endl;
        std::map<ObjectType, Shape3DData>::iterator it, it_end = current_blob->occ_data->end();
        for(it = current_blob->occ_data->begin(); it != it_end; it++) {
            out << "\t\t\t\t\t\t" << " Type :" << Blob::getNameFromType(it->first) << std::endl;
            out << it->second << std::endl;
        }
    }

    return out;
}


//Rendering Functions
std::ostream& operator<<(std::ostream& out, const Shape3DData& ss) {
    const Shape3DData *current_s = &ss;
    out << "\t\t\t\t\tS3D (type,subtype,P,w,l,h,alpha) : (";
    out << Blob::getNameFromType(S3D_TYPE(current_s)) << ", " << Blob::getNameFromSubtype(S3D_SUBTYPE(current_s)) << ", " << S3D_P(current_s) << ", ";
    out << S3D_W(current_s) << ", " << S3D_L(current_s) << ", "<<S3D_H(current_s);
    out << ", "<<S3D_ALPHA(current_s)<<")" << std::endl;
    out << "\t\t\t\t\t\t Occlusion: " << Blob::getDPNameFromTypeDetailed(S3D_DP_TYPE(current_s)) << std::endl;
    out << "\t\t\t\t\t\t (x,y,z)           : (" << S3D_3D_X(current_s);
    out << ", " <<S3D_3D_Y(current_s) << ", " << S3D_3D_Z(current_s) << ")" << std::endl;
    out << "\t\t\t\t\t\t" << " (W,L) : (" << S3D_WIDTH(current_s) << ", " << S3D_HEIGHT(current_s) << ")" << std::endl;
    out << "\t\t\t\t\t\t" << " (X,Y) : (" << S3D_XCENTER(current_s) << ", " << S3D_YCENTER(current_s) << ")" << std::endl;
    out << "\t\t\t\t\t\t" << " maxDistanceFactor : " <<  current_s->maxDistanceFactor << std::endl;

    if(current_s->subtypesList == NULL)
        out << "\t\t\t\t\t" << " Subtypes Data : NULL" << std::endl;
    else {
        out << "\t\t\t\t\t" << " Subtypes Data :" << std::endl;
        std::map<ObjectSubtype, Shape3DData>::iterator it, it_end = current_s->subtypesList->end();
        for(it = current_s->subtypesList->begin(); it != it_end; it++) {
            out << "\t\t\t\t\t\t" << " Subtype :" << Blob::getNameFromSubtype(it->first) << std::endl;
            out << it->second << std::endl;
        }
    }

    return out;
}

blobBuffer::blobBuffer(int i_maxLength) : maxLength(i_maxLength) {
    if(maxLength <= 0) maxLength = 1;
}

blobBuffer::~blobBuffer(){
    trackedBlobs.clear();
}

std::deque<SpBlob>::iterator blobBuffer::begin() {
    return trackedBlobs.begin();
}

std::deque<SpBlob>::iterator blobBuffer::end() {
    return trackedBlobs.end();
}

std::deque<SpBlob>::reverse_iterator blobBuffer::rbegin() {
    return trackedBlobs.rbegin();
}

std::deque<SpBlob>::reverse_iterator blobBuffer::rend() {
    return trackedBlobs.rend();
}

SpBlob blobBuffer::operator[](int index){
    return trackedBlobs[index];
}

SpBlob blobBuffer::back() {
    return trackedBlobs.back();
}

unsigned int blobBuffer::size() {
    return trackedBlobs.size();
}

void blobBuffer::insert(Blob *blob) {
    SpBlob newTBlob(new Blob(blob));

    if(size() == maxLength)
        trackedBlobs.pop_front();

    trackedBlobs.push_back(newTBlob);
}

void blobBuffer::clear() {
    trackedBlobs.clear();
}

void blobBuffer::resetMaxLength(int i_maxLength) {
    maxLength = (i_maxLength <= 0) ? 1: i_maxLength;
}

void blobBuffer::copyBlobs(blobBuffer *buffer) {
    trackedBlobs = buffer->trackedBlobs;
}

//Rendering Functions
std::ostream& operator<<(std::ostream& out, const blobBuffer& bb) {
    std::deque<SpBlob> *blobs = (std::deque<SpBlob> *)&bb.trackedBlobs;
    SpBlob current_blob;
    std::deque<SpBlob>::iterator it;
    int i = 0;
    for(it = blobs->begin(); it != blobs->end(); it++, i++){
        current_blob = *it;
        out << "\t\t\t\tBlob " << i << " (ID,type,P,w,l,h,alpha) : (" << BLOB_IDENT(current_blob) << ", ";
        out << Blob::getNameFromType(BLOB_TYPE(current_blob)) << ", " << BLOB_P(current_blob) << ", ";
        out << BLOB_3D_WIDTH(current_blob) << ", " << BLOB_3D_LENGTH(current_blob) << ", "<<BLOB_3D_HEIGHT(current_blob);
        out << ", "<<BLOB_ALPHA(current_blob)<<")" << std::endl;
        out << "\t\t\t\t\t (x,y,z)           : (" << BLOB_3D_X(current_blob);
        out << ", " <<BLOB_3D_Y(current_blob) << ", " << BLOB_3D_Z(current_blob) << ")" << std::endl;
        out << "\t\t\t\t\t" << " (W,L) : (" << BLOB_WIDTH(current_blob) << ", " << BLOB_HEIGHT(current_blob) << ")" << std::endl;
        out << "\t\t\t\t\t" << " (X,Y) : (" << BLOB_XCENTER(current_blob) << ", " << BLOB_YCENTER(current_blob) << ")" << std::endl;
    }
    return out;
}

