#ifndef LEARNING_ATTRIBUTES_SPECIFICATION_H
#define LEARNING_ATTRIBUTES_SPECIFICATION_H

#include "RMMT/RMMMobileObject.h" //Class representing a tracked mobile object
typedef SpRMMMobileObject SpMObject;

class Datapool;

#include <map>
#include <set>
#include <deque>
#include <string>

namespace miles {

//File for defining new attributes for event learning: Search for the word Step for Knowing what to include... :D
//ATTENTION!! The  Steps must be properly accomplished for a correct consideration of the attribute.

//Prototype function pointers
typedef double (*fp_getDouble)(SpMObject, Datapool&);
typedef std::string (*fp_getString)(SpMObject, Datapool&);
typedef std::set<std::string> *(*fp_setSymValues)();

//Step 1. First define a new entry in the enum list of attribute ids, regardless if type is numerical or symbolic
enum attribute_id {
    LA_X,
    LA_Y,
    LA_VX,
    LA_VY,
    LA_V2D,
    LA_W,
    LA_H,
    LA_VW,
    LA_VH,
    LA_DKITCHEN, //Distance of the object to the context object "kitchen"
    LA_DTABLE,   //Distance of the object to the context object "table"
    LA_DSOFA,    //Distance of the object to the context object "sofa"
    LA_SKITCHEN, //Symbolic distance of the object to the context object "kitchen"
    LA_STABLE,   //Symbolic distance of the object to the context object "table"
    LA_SSOFA,    //Symbolic distance of the object to the context object "sofa"
    LA_HX,       //Homography X position
    LA_HY,       //Homography Y position
    LA_DEGU_CORNER, //Symbolic position of DEGU in corners.
};


//Predefine every class to avoid cyclic reference problems
class LearningContextualisedObject;
class XMLTreeGenerator;
class LearningAttributes;
class LearningStateConcept;
class LearningEventConcept;
class LearningContext;
class HierarchyTree;
class LevelData;
class ContextualisedNumericalData;
class ContextualisedSymbolicData;
class NumericalAttributeValuePair;
class SymbolicAttributeValuePair;

typedef QSharedPointer<LearningContextualisedObject> SpLearningContextualisedObject;
typedef QSharedPointer<XMLTreeGenerator> SpXMLTreeGenerator;
typedef QSharedPointer<LearningAttributes> SpLearningAttributes;
typedef QSharedPointer<LearningStateConcept> SpLearningStateConcept;
typedef QSharedPointer<LearningEventConcept> SpLearningEventConcept;
typedef QSharedPointer<LearningContext> SpLearningContext;
typedef QSharedPointer<HierarchyTree> SpHierarchyTree;
typedef QSharedPointer<LevelData> SpLevelData;
typedef QSharedPointer<ContextualisedNumericalData> SpContextualisedNumericalData;
typedef QSharedPointer<ContextualisedSymbolicData> SpContextualisedSymbolicData;
typedef QSharedPointer<NumericalAttributeValuePair> SpNumericalAttributeValuePair;
typedef QSharedPointer<SymbolicAttributeValuePair> SpSymbolicAttributeValuePair;



// Step 2: Define the class interface specification for the attribute (follow examples).
//         Make only prototypes, the implementation goes in file: CLearningAttributesSpecification.cpp       

//Specification for numerical attributes:

//2D Spatial Attributes
class HX {
public:
  static double getValue(SpMObject mobile, Datapool& datapool);
  static double getReliability(SpMObject mobile, Datapool& datapool);
};

class HY {
public:
  static double getValue(SpMObject mobile, Datapool& datapool);
  static double getReliability(SpMObject mobile, Datapool& datapool);
};


  //2D Spatial Attributes
  class X {
  public:
    static double getValue(SpMObject mobile, Datapool& datapool);
    static double getReliability(SpMObject mobile, Datapool& datapool);
  };

  class Y {
  public:
    static double getValue(SpMObject mobile, Datapool& datapool);
    static double getReliability(SpMObject mobile, Datapool& datapool);
  };

  class VX {
  public:
    static double getValue(SpMObject mobile, Datapool& datapool);
    static double getReliability(SpMObject mobile, Datapool& datapool);
  };

  class VY {
  public:
    static double getValue(SpMObject mobile, Datapool& datapool);
    static double getReliability(SpMObject mobile, Datapool& datapool);
  };

  class V2D {
  public:
    static double getValue(SpMObject mobile, Datapool& datapool);
    static double getReliability(SpMObject mobile, Datapool& datapool);
  };

  //2D Dimensional Attributes
  class W {
  public:
    static double getValue(SpMObject mobile, Datapool& datapool);
    static double getReliability(SpMObject mobile, Datapool& datapool);
  };

  class H {
  public:
    static double getValue(SpMObject mobile, Datapool& datapool);
    static double getReliability(SpMObject mobile, Datapool& datapool);
  };

  class VW {
  public:
    static double getValue(SpMObject mobile, Datapool& datapool);
    static double getReliability(SpMObject mobile, Datapool& datapool);
  };

  class VH {
  public:
    static double getValue(SpMObject mobile, Datapool& datapool);
    static double getReliability(SpMObject mobile, Datapool& datapool);
  };



  class DKITCHEN { //Distance of the object to the context object "kitchen"
  public:
    static double getValue(SpMObject mobile, Datapool& datapool);
    static double getReliability(SpMObject mobile, Datapool& datapool);
  };

  class DTABLE {   //Distance of the object to the context object "table"
  public:
    static double getValue(SpMObject mobile, Datapool& datapool);
    static double getReliability(SpMObject mobile, Datapool& datapool);
  };

  class DSOFA {    //Distance of the object to the context object "sofa"
    public:
    static double getValue(SpMObject mobile, Datapool& datapool);
    static double getReliability(SpMObject mobile, Datapool& datapool);
  };
  
  //Specification for symbolic attributes:
  class DEGU_CORNER { //Symbolic distance of the object to the context object "kitchen"
  public:
    static std::set<std::string> values;
    static bool checked, valid;
    static std::map< std::string, QSharedPointer<world::ZoneH> > zones;
    static std::string getValue(SpMObject mobile, Datapool& datapool);
    static double getReliability(SpMObject mobile, Datapool& datapool);
    static std::set<std::string> *setAllowedValues();
  };


  class SKITCHEN { //Symbolic distance of the object to the context object "kitchen"
  public:
    static std::set<std::string> values;
    static std::string getValue(SpMObject mobile, Datapool& datapool);
    static double getReliability(SpMObject mobile, Datapool& datapool);
    static std::set<std::string> *setAllowedValues();
  };

  class STABLE {   //Symbolic distance of the object to the context object "table"
  public:
    static std::set<std::string> values;
    static std::string getValue(SpMObject mobile, Datapool& datapool);
    static double getReliability(SpMObject mobile, Datapool& datapool);
    static std::set<std::string> *setAllowedValues();
  };
  
  class SSOFA {    //Symbolic distance of the object to the context object "sofa"
  public:
    static std::set<std::string> values;
    static std::string getValue(SpMObject mobile, Datapool& datapool);
    static double getReliability(SpMObject mobile, Datapool& datapool);
    static std::set<std::string> *setAllowedValues();
  };
  

  //Class summarising all the attribute specifications
  class LearningAttributes {
  public:
    LearningAttributes(){}
    ~LearningAttributes(){}
    //General lists of data extraction functions
    std::map<attribute_id, std::string> nameFromId;
    std::map<std::string, attribute_id> idFromName;

    std::set<std::string> availableNumerical;
    std::set<std::string> availableSymbolic;

    std::map<attribute_id, fp_getDouble> getValueNumeric;
    std::map<attribute_id, fp_getDouble> getReliability;
    std::map<attribute_id, fp_getString> getValueSymbolic;
    std::map<attribute_id, std::set<std::string>*> symbolicValuesList;

    void setNumericalAttribute(attribute_id i_id, std::string i_name, fp_getDouble getValueFunction, fp_getDouble getReliabilityFunction) {
        availableNumerical.insert(i_name);
        nameFromId[i_id] = i_name;
        idFromName[i_name] = i_id;
        getValueNumeric[i_id] = getValueFunction;
        getReliability[i_id] = getReliabilityFunction;
    }

    void setSymbolicAttribute(attribute_id i_id, std::string i_name, fp_getString getValueFunction, fp_getDouble getReliabilityFunction, fp_setSymValues setSymbolicValues) {
        availableSymbolic.insert(i_name);
        nameFromId[i_id] = i_name;
        idFromName[i_name] = i_id;
        getValueSymbolic[i_id] = getValueFunction;
        getReliability[i_id] = getReliabilityFunction;
        symbolicValuesList[i_id] = (*setSymbolicValues)();
    }

    //Step 3. Insert the specification of the attribute.
 
    //   In this function, define the numerical attributes: 
    //   the id defined in Step 1, the name to link with predefined learning contexts, 
    //   the function for getting the attribute value from data,
    //   and the function for getting the attribute's reliability from data,
    void setNumericalAttributes() {
        setNumericalAttribute(LA_X,  "X",  &X::getValue,  &X::getReliability);
        setNumericalAttribute(LA_Y,  "Y",  &Y::getValue,  &Y::getReliability);
        setNumericalAttribute(LA_HX,  "HX",  &HX::getValue,  &HX::getReliability);
        setNumericalAttribute(LA_HY,  "HY",  &HY::getValue,  &HY::getReliability);
        setNumericalAttribute(LA_VX, "VX", &VX::getValue, &VX::getReliability);
        setNumericalAttribute(LA_VY, "VY", &VY::getValue, &VY::getReliability);
        setNumericalAttribute(LA_V2D,"V2D",&V2D::getValue,  &V2D::getReliability);
        setNumericalAttribute(LA_W,  "W",  &W::getValue,  &W::getReliability);
        setNumericalAttribute(LA_H,  "H",  &H::getValue,  &H::getReliability);
        setNumericalAttribute(LA_VW, "VW", &VW::getValue, &VW::getReliability);
        setNumericalAttribute(LA_VH, "VH", &VH::getValue, &VH::getReliability);
        setNumericalAttribute(LA_DKITCHEN, "DKITCHEN",  &DKITCHEN::getValue,  &DKITCHEN::getReliability);
        setNumericalAttribute(LA_DTABLE,   "DTABLE",  &DTABLE::getValue,  &DTABLE::getReliability);
        setNumericalAttribute(LA_DSOFA,    "DSOFA",  &DSOFA::getValue,  &DSOFA::getReliability);
    }

    //   In this function, define the symbolic attributes: 
    //   the id defined in Step 1, the name to link with predefined learning contexts, 
    //   the function for getting the attribute value from data,
    //   the function for getting the attribute's reliability from data,
    //   and the function setting and returning the list of valid symbolic attribute values
    void setSymbolicAttributes() {
        setSymbolicAttribute(LA_SKITCHEN,   "SKITCHEN",  &SKITCHEN::getValue,  &SKITCHEN::getReliability, &SKITCHEN::setAllowedValues);
        setSymbolicAttribute(LA_STABLE,     "STABLE",  &STABLE::getValue,  &STABLE::getReliability, &STABLE::setAllowedValues);
        setSymbolicAttribute(LA_SSOFA,      "SSOFA",  &SSOFA::getValue,  &SSOFA::getReliability, &SSOFA::setAllowedValues);
        setSymbolicAttribute(LA_DEGU_CORNER, "DEGU_CORNER",&DEGU_CORNER::getValue, &DEGU_CORNER::getReliability, &DEGU_CORNER::setAllowedValues);
    }
  };
  
  typedef QSharedPointer<LearningAttributes> SpLearningAttributes;

} // end namespace

#endif	
