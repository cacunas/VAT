#ifndef __LEARNING_CONCEPT_H__
#define __LEARNING_CONCEPT_H__

#include <set>
#include <string>
#include "LearningAttributesSpecification.h"
#include "GaussianFunction.h"

namespace miles {

  class NumericalAttributeValuePair {

    public:   
    attribute_id id;
    SpGaussianFunction distribution;
    double p;
    double RSum; //Accumulated sum of reliabilities 

    /*Constructor for numerical value type*/
    NumericalAttributeValuePair(attribute_id, double, double, double);
    ~NumericalAttributeValuePair();
    
    void setId(attribute_id i_id);
    attribute_id getId();
    void modifyValue(double mean, double sigma);

    bool operator==(NumericalAttributeValuePair);

    double getMean();
    double getSigma();
    double getRSum();

    void setMean(double mean);
    void setSigma(double sigma);
    void setRSum(double i_RSum);

    double getP();
    void setP(double);

    void print(SpLearningAttributes availableAttributes);
    
  };

  class SymbolicValue {
  public:
    double RSum;
    double P;
    int N;
    SymbolicValue(double, double, int);
    SymbolicValue();
    ~SymbolicValue();
  };

  typedef QSharedPointer<SymbolicValue> SpSymbolicValue;


  class SymbolicAttributeValuePair {

    public:   
    attribute_id id;
    double p;
    double RSum; //Accumulated sum of reliabilities 
    std::map<std::string, SpSymbolicValue> values;

    /*Constructor for symbolic value type*/
    SymbolicAttributeValuePair(attribute_id, double, std::map<std::string, SpSymbolicValue> *);
    SymbolicAttributeValuePair(attribute_id, double, std::string, std::set<std::string> *);
    ~SymbolicAttributeValuePair();
    
    void setId(attribute_id i_id);
    attribute_id getId();
    SpSymbolicValue modifyValue(std::string value, double R);

    bool operator==(SymbolicAttributeValuePair);

    SpSymbolicValue getValueInfo(std::string);
    double getValueRSum(std::string);
    double getValueP(std::string);
    int getValueN(std::string);

    double getRSum();
    void setRSum(double i_RSum);

    double getP();
    void setP(double);

    void print(SpLearningAttributes availableAttributes);

  };

  class LearningEventConcept {
  public:
    long int id;
    int N;
    double P;
    SpGaussianFunction timeStayingAtOutgoingState;
    double minT, maxT;

    long int startingState;
    long int arrivingState;

    double getMean();
    double getSigma();

    LearningEventConcept(long int i_id, double mean, double sigma);
    ~LearningEventConcept();

    void setMean(double mean);
    void setSigma(double sigma);

  };

  class LearningStateConcept {
  public:
    long int id;
    std::map <attribute_id, SpNumericalAttributeValuePair> numericalAttributeList;
    std::map <attribute_id, SpSymbolicAttributeValuePair> symbolicAttributeList;

    //Indexed by the id of the state of origin
    std::map <long int, SpLearningEventConcept> incomingEvents;
    //Indexed by the id of the arriving state
    std::map <long int, SpLearningEventConcept> outgoingEvents;

    int N;
    int NE;
    double P;
    double score; //category utility score
    double diff;  //attributes difference score
    long int parent;
    int level, depth, distance_to_leaf;
    LearningStateConcept();
    LearningStateConcept(double);
    ~LearningStateConcept();

    SpNumericalAttributeValuePair insertNumericalPair(attribute_id, double, double, double);

    SpSymbolicAttributeValuePair insertSymbolicPair(attribute_id, double, std::map<std::string, SpSymbolicValue> *);
    SpSymbolicAttributeValuePair insertSymbolicPair(attribute_id, double, std::string, std::set<std::string> *values);

    void removePair(attribute_id);

    void print(SpLearningAttributes availableAttributes);

    bool numericalIdExists(attribute_id);
    bool symbolicIdExists(attribute_id);

    void removeNumericalPair(attribute_id);
    void removeSymbolicPair(attribute_id);

    SpNumericalAttributeValuePair getNumericPair(attribute_id);
    SpSymbolicAttributeValuePair getSymbolicPair(attribute_id);

    int numericalPairsListSize();
    int symbolicPairsListSize();

    double getP();
    void setP(double);

    int getN();
    void setN(int);

    double getScore();
    void setScore(double);

    double getDiff();
    void setDiff(double);

    long int getID();
    void setID(long int);

    int getParent();
    void setParent(int);

    double getPositionP(attribute_id _x, attribute_id _y);

    void setAttributeConceptToArray(double *Ps, int w, int h, attribute_id _x, attribute_id _y, attribute_id _a, double maxA, double ignorePThreshold);
    void setConceptToArray(double *Ps, int w, int h, attribute_id _x, attribute_id _y, double maxP, double ignorePThreshold);
    void setConceptBorderToArray(double *Ps, int w, int h, attribute_id _x, attribute_id _y, double maxP, double ignorePThreshold);
    void setFlatConceptToArray(double *Ps, int w, int h, attribute_id _x, attribute_id _y, double maxP, double ignorePThreshold);
  };

} // end namespace

#endif
