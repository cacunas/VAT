#include "reliabilityattribute.h"

AttributeDupletValue::AttributeDupletValue(): value(0.0), RD(0.0) { }

AttributeDupletValue::AttributeDupletValue(double v, double R): value(v), RD(R) { }

AttributeDupletValue::~AttributeDupletValue() {}

AttributeDupletImage::AttributeDupletImage():
    image(NULL), RD(1.0),
    Rdataset_active(false), Rdataset(NULL) { }

AttributeDupletImage::AttributeDupletImage(QImage *im, double R):
    image(im), RD(R),
    Rdataset_active(false), Rdataset(NULL) { }

AttributeDupletImage::AttributeDupletImage(QImage *im, cv::Mat *Rdata):
    image(im), RD(1.0),
    Rdataset_active(true), Rdataset(Rdata) { }

AttributeDupletImage::~AttributeDupletImage() {
    if(image != NULL) delete image;
    if(Rdataset != NULL) delete Rdataset;
}

AttributeDupletDataset::AttributeDupletDataset():
    RD(1.0), Rdataset_active(false), Rdataset(NULL) { }

AttributeDupletDataset::AttributeDupletDataset(cv::Mat &data, double R):
    dataset(data), RD(R),
    Rdataset_active(false), Rdataset(NULL) { }

AttributeDupletDataset::AttributeDupletDataset(cv::Mat &dataset, cv::Mat *Rdata):
    RD(1.0), Rdataset_active(true), Rdataset(Rdata) {
    dataset.copyTo(this->dataset);
}


AttributeDupletDataset::~AttributeDupletDataset() {
    if(Rdataset != NULL) delete Rdataset;
}



AttributeTripletValue::AttributeTripletValue() {}
AttributeTripletValue::~AttributeTripletValue() {}

AttributeTripletDataset::AttributeTripletDataset():Rdataset_active(false), DR(NULL) {}
AttributeTripletDataset::~AttributeTripletDataset() {
    if(DR != NULL) delete DR;
}

ReliabilityDynamicsAttribute::ReliabilityDynamicsAttribute() :
updateAtt(true), updateV(true), updateA(true)
{ }

ReliabilityDynamicsAttribute::ReliabilityDynamicsAttribute(bool at, bool v, bool a) :
updateAtt(at), updateV(v), updateA(a)
{ }

void ReliabilityDynamicsAttribute::init(bool at, bool v, bool a) {
    updateAtt = at; updateV = v; updateA = a;
}

ReliabilityDynamicsAttributeDataset::ReliabilityDynamicsAttributeDataset() :
updateAtt(true), updateV(true), updateA(true)
{ }

ReliabilityDynamicsAttributeDataset::ReliabilityDynamicsAttributeDataset(bool at, bool v, bool a) :
updateAtt(at), updateV(v), updateA(a)
{ }

void ReliabilityDynamicsAttributeDataset::init(bool at, bool v, bool a) {
    updateAtt = at; updateV = v; updateA = a;
}

