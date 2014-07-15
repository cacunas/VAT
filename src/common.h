#ifndef COMMON_H
#define COMMON_H

enum ObjectType {
    UNKNOWN       = 0,    // blob type unknown
    CROWD         = 1,     // blob contains many many persons
    PERSON_GROUP  = 2,     // may be a person, maybe a group
    A_GROUP       = 3,     // blob is a group
    PERSON        = 4,     // blob is a person
    CONTX_OBJECT  = 5,     // blob is a context object, ex: the door
    VEHICLE       = 6,     // blob is a vehicle, ex: metro, car
    NOISE         = 7,     // noise
    CAR           = 9,
    TRUCK         = 10,
    MOTORCYCLE    = 11,
    PART_OF_GROUP = 12,        // blob is a part of a group
    PART_OF_PERSON_GROUP = 13, // blob is a part of a person_group
    UNDETERMINATE        = 14, // blob type is indeterminate
    CHILD                = 15,
    PARKED_VEHICLE       = 16,
    FACE       = 17,
    TWOPERSONS = 18,
    THREEPERSONS = 19,
    PART_OF_PERSON =20,        // blob is a part of person
    ANY2D = 21,                // For learning context: Any class even UNKNOWN
    ANY3D = 22,                // For learning context: Any class without UNKNOWN
    DEGU  = 23                 //Type for DeguObjectClass
};


enum ObjectSubtype {
    ST_UNKNOWN      = 0,  // blob subtype unknown
    ST_UNDEFINED    = 1,  // blob subtype undefined
    ST_NO_SUBTYPES  = 2,  // constant for rigid objects
    ST_STANDING     = 3,  // Standing posture
    ST_CROUCHING    = 4,  // Crouching posture
    ST_LYING        = 5,  // Lying posture
};

//Coding by binary flags for blob detection problems to allow the representation of multiple types of occlusion at the same time
enum DetectionProblemType {
    MM_DP_NONE               = 0x00000000,//00000000000000000000000000000000,  // no detection problem
    MM_CAM_OCCL_BOTTOM       = 0x00000001,//00000000000000000000000000000001,  // only occlusion on image bottom
    MM_CAM_OCCL_TOP          = 0x00000002,//00000000000000000000000000000010,  // only occlusion on image top
    MM_CAM_OCCL_LEFT         = 0x00000004,//00000000000000000000000000000100,  // only occlusion on image left
    MM_CAM_OCCL_RIGHT        = 0x00000008,//00000000000000000000000000001000,  // only occlusion on image right
    MM_CAM_OCCL_BOTTOM_LEFT  = 0x00000005,//00000000000000000000000000000101,  // occlusion on bottom and left corner of image
    MM_CAM_OCCL_BOTTOM_RIGHT = 0x00000009,//00000000000000000000000000001001,  // occlusion on bottom and right corner of image
    MM_CAM_OCCL_TOP_LEFT     = 0x00000006,//00000000000000000000000000000110,  // occlusion on top and left corner of image
    MM_CAM_OCCL_TOP_RIGHT    = 0x0000000A,//00000000000000000000000000001010,  // occlusion on top and right corner of image
    MM_CAM_OCCL_MASK         = 0x0000000F,//00000000000000000000000000001111,  // camera occlusion mask
    MM_OBJECT_BOTTOM         = 0x00000010,//00000000000000000000000000010000,  // blob is occluded on bottom part by a context object
    MM_OBJECT_TOP            = 0x00000020,//00000000000000000000000000100000,  // blob is occluded on top part by a context object
    MM_OBJECT_LEFT           = 0x00000040,//00000000000000000000000001000000,  // blob is occluded on left part by a context object
    MM_OBJECT_RIGHT          = 0x00000080,//00000000000000000000000010000000,  // blob is occluded on right part by a context object
    MM_OBJECT_BOTTOM_LEFT    = 0x00000050,//00000000000000000000000001010000,  // blob is occluded on bottom-left part by a context object
    MM_OBJECT_BOTTOM_RIGHT   = 0x00000090,//00000000000000000000000010010000,  // blob is occluded on bottom-right part by a context object
    MM_OBJECT_TOP_LEFT       = 0x00000060,//00000000000000000000000001100000,  // blob is occluded on top-left part by a context object
    MM_OBJECT_TOP_RIGHT      = 0x000000A0,//00000000000000000000000010100000,  // blob is occluded on top-right part by a context object
    MM_OBJECT_OCCL_MASK      = 0x000000F0,//00000000000000000000000011110000,  // context object occlusion mask
    MM_DYNAMIC_BOTTOM        = 0x00000100,//00000000000000000000000100000000,  // blob is occluded on bottom part by a mobile object
    MM_DYNAMIC_TOP           = 0x00000200,//00000000000000000000001000000000,  // blob is occluded on top part by a mobile object
    MM_DYNAMIC_LEFT          = 0x00000400,//00000000000000000000010000000000,  // blob is occluded on left part by a mobile object
    MM_DYNAMIC_RIGHT         = 0x00000800,//00000000000000000000100000000000,  // blob is occluded on right part by a mobile object
    MM_DYNAMIC_BOTTOM_LEFT   = 0x00000500,//00000000000000000000010100000000,  // blob is occluded on bottom-left part by a mobile object
    MM_DYNAMIC_BOTTOM_RIGHT  = 0x00000900,//00000000000000000000100100000000,  // blob is occluded on bottom-right part by a mobile object
    MM_DYNAMIC_TOP_LEFT      = 0x00000600,//00000000000000000000011000000000,  // blob is occluded on top-left part by a mobile object
    MM_DYNAMIC_TOP_RIGHT     = 0x00000A00,//00000000000000000000101000000000,  // blob is occluded on top-right part by a mobile object
    MM_DYNAMIC_OCCL_MASK     = 0x00000F00,//00000000000000000000111100000000,  // dynamic occlusion mask
    MM_HORIZONTAL_OCCL_MASK  = 0x000000CC,//00000000000000000000000011001100,  // horizontal static occlusion mask
    MM_VERTICAL_OCCL_MASK    = 0x00000033,//00000000000000000000000000110011,  // vertical static occlusion mask
    MM_HORIZONTAL_DYN_MASK   = 0x00000CCC,//00000000000000000000110011001100,  // horizontal dynamic occlusion mask
    MM_VERTICAL_DYN_MASK     = 0x00000333,//00000000000000000000001100110011,  // vertical dynamic occlusion mask
    MM_BOTTOM_OCCL           = 0x00000111,//00000000000000000000000100010001,  // bottom occlusion
    MM_TOP_OCCL              = 0x00000222,//00000000000000000000001000100010,  // top occlusion
    MM_LEFT_OCCL             = 0x00000444,//00000000000000000000010001000100,  // left occlusion
    MM_RIGHT_OCCL            = 0x00000888,//00000000000000000000100010001000,  // right occlusion
    MM_TOTAL_OCCLUSION       = 0x00010000,//00000000000000010000000000000000   // total occlusion of an object
    MM_OBJECT_LOST           = 0x00020000,//00000000000000100000000000000000,  // blob is just an estimation based on previously obtained information, its data is unreliable
    MM_NOT_VISIBLE_MASK      = 0x00030000,//00000000000000110000000000000000,  // not visible blob, lost or totally occluded
    MM_PARTIALLY_DETECTED    = 0x00040000,//00000000000001000000000000000000   // total occlusion of an object
    MM_PART_OF_BIGGER        = 0x00080000,//00000000000010000000000000000000   // total occlusion of an object
    MM_NOT_3D_TRACKABLE_MASK = 0x000C0000,//00000000000011000000000000000000,  // not visible blob, lost or totally occluded
    MM_AOI_OUTSIDE           = 0x02000000,//00000010000000000000000000000000,  // blob is outside aoi ground
    MM_AOI_REINSIDE          = 0x04000000,//00000100000000000000000000000000,  // blob is re-inside aoi ground
    MM_OBJECT_INSIDE         = 0x08000000,//00001000000000000000000000000000,  // blob is inside object
};

typedef double perspective_matrix[12];
typedef double homography_matrix[9];

#endif // COMMON_H
