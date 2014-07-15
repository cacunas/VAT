#ifndef LightSourceModelDYNAMICS_H
#define LightSourceModelDYNAMICS_H

#include <QDomNode>
#include <QImage>
#include <vector>
#include <QSharedPointer>

#include "reliabilitysinglemodelinterface.h"
#include "src/blob.h"
#include "Blob2DFromBGSubstractionModel.h"

class LightSourceModel: public Blob2DFromBGSubstractionModel
{
public:
    LightSourceModel();

    ~LightSourceModel();







    double m_minimalAttributeVelocityReliability;
    double m_minimalAttributeAccelerationReliability;

    int m_pixelAcuity;
};


#endif // LightSourceModelDYNAMICS_H
