#ifndef RELIABILITYATTRIBUTE_H
#define RELIABILITYATTRIBUTE_H

#include <QString>
#include <QImage>
#include <opencv2/opencv.hpp>


//Attributes for model instance, with only current data and reliability
class AttributeDupletValue {
public:
    AttributeDupletValue();
    AttributeDupletValue(double v, double R);   //A value with single reliability value
    ~AttributeDupletValue();
    double value;
    double RD; //reliability from quality of data, derived from input.
};

class AttributeDupletImage {
public:
    AttributeDupletImage();
    AttributeDupletImage(QImage *image, double R);     //An image with single reliability value
    AttributeDupletImage(QImage *image, cv::Mat *Rdataset); //An image with multiple reliability values
    ~AttributeDupletImage();

    QImage *image;

    double RD;

    bool Rdataset_active; //true if reliability dataset active
    cv::Mat *Rdataset;
};

class AttributeDupletDataset {
public:
    AttributeDupletDataset();
    AttributeDupletDataset(cv::Mat &dataset, double R); //Multiple values with single reliability value
    AttributeDupletDataset(cv::Mat &dataset, cv::Mat *Rdataset); //Multiple values with multiple reliability values
    ~AttributeDupletDataset();
    cv::Mat dataset;

    double RD;

    bool Rdataset_active; //true if reliability dataset active
    cv::Mat *Rdataset;
};



class ValueReliability {
public:
   double RD;
   double RC;
   double SD;
   double accR;
};

class DatasetReliability {
public:
    cv::Mat RD;
    cv::Mat RC;
    cv::Mat SD;
    cv::Mat accR;
};

class AttributeTripletValue { //Triplet: Value; VR.RC; VR.RD
public:
    AttributeTripletValue();
    ~AttributeTripletValue();

    double value;
    ValueReliability VR;
};

//For an image, dynamics should also be accumulated in a dataset (for precision)
class AttributeTripletDataset {
public:
    AttributeTripletDataset();
    ~AttributeTripletDataset();

    cv::Mat dataset;

    bool Rdataset_active; //true when dataset reliability active

    ValueReliability VR;
    DatasetReliability *DR;

};


class ReliabilityDynamicsAttribute
{
public:
    ReliabilityDynamicsAttribute();
    ReliabilityDynamicsAttribute(bool at, bool v, bool a);
    void init(bool at, bool v, bool a);

    bool updateAtt; //Consider that the attribute can change (not just vary with error)
    bool updateV;   //Consider dynamic velocity
    bool updateA;   //Consider dynamic acceleration

    AttributeTripletValue att; //attribute value + reliabilities + aux data
    AttributeTripletValue V;   //velocity + reliabilities + aux data
    AttributeTripletValue A;   //acceleration + reliabilities + aux data
};

class ReliabilityDynamicsAttributeDataset
{
public:
    ReliabilityDynamicsAttributeDataset();
    ReliabilityDynamicsAttributeDataset(bool at, bool v, bool a);
    void init(bool at, bool v, bool a);

    bool updateAtt; //Consider that the attribute can change (not just vary with error)
    bool updateV;   //Consider dynamic velocity
    bool updateA;   //Consider dynamic acceleration

    AttributeTripletDataset att; //attribute value + reliabilities + aux data
    AttributeTripletDataset V;   //velocity + reliabilities + aux data
    AttributeTripletDataset A;   //acceleration + reliabilities + aux data
};



#endif // RELIABILITYATTRIBUTE_H
