#ifndef OBJECTMODELREADER_H
#define OBJECTMODELREADER_H

#include "QtXml/QDomDocument"
#include <QtXml/QDomElement>
#include <QtXml/QDomText>
#include<QString>
#include"Datapool.h"
#include"ModelInterface.h"

class ObjectModelReader {
public:
    QString m_fileName;
    Datapool *m_data;
    QDomDocument *xmlObjectModels;
    ObjectModelReader(Datapool *i_data);
    bool readObjectModels(QString fileName);
    bool readParameters(QString fileName);
    //Model reading
    bool readParallelpipedModel(SpModelInterface model, QDomNode& n);


};

#endif // OBJECTMODELREADER_H
