#include <QFile>
#include "ObjectModelReader.h"
#include "GaussianFunction.h"
#include "VideoAnalysis.h"
#include "blob.h"

ObjectModelReader::ObjectModelReader(Datapool *i_data): m_data(i_data) {
    xmlObjectModels = NULL;
}

bool ObjectModelReader::readObjectModels(QString fileName) {
    if(xmlObjectModels != NULL)
        delete xmlObjectModels;
    xmlObjectModels = new QDomDocument( "ObjectModels" );
    QFile file( fileName );
    if( !file.open(QIODevice::ReadOnly) ) {
        AppendToLog("ObjectModelReader: Error opening file. The object models file '" + fileName + "' could not be opened. Execution will be aborted.");
        return false;
    }

    QString error;
    int line, column;
    if( !xmlObjectModels->setContent( &file ) ) {
        AppendToLog("ObjectModelReader: Error opening file. XML content in the object models file '" + fileName + "' could not be extracted. Execution will be aborted.");
        file.close();
        return false;
    }
    file.close();

    return readParameters(fileName);

    return true;
}

bool ObjectModelReader::readParameters(QString fileName) {

    m_fileName = fileName;

    QDomElement root = xmlObjectModels->documentElement();
    if( root.tagName() != "ObjectModels" ) {
        AppendToLog("ObjectModelReader: Error reading XML information. XML content in the object models file '" + fileName + "' does not correspond to an object model description. Execution will be aborted.");
        return false;
    }

    std::map<ObjectType, SpModelInterface>& objectModels = m_data->objectModels;

    QDomNode n = root.firstChild(), m;
    if(!n.isNull())
        objectModels.clear();

    QDomElement e;
    ObjectType type;
    QString aux, omodel;
    bool rigid;
    while( !n.isNull() ) {
        e = n.toElement();
        SpModelInterface model(new ModelInterface(m_data));
        if( !e.isNull() ) {
            if( e.tagName() == "ObjectModel" ) {

                if( (aux = e.attribute( "name", "" )) == "") {
                    AppendToLog("ObjectModelReader: Error reading XML information. Undefined attribute 'name' for ObjectModel in file '" + fileName + "'. Execution will be aborted.");
                    return false;
                }
                type = Blob::getTypeFromName(aux.toStdString());
                if(type == UNKNOWN) {
                    AppendToLog("ObjectModelReader: Error reading XML information. Attribute 'name' does not correspond to any pre-defined type. Check ObjectType in file 'src/common.h' for allowed types. Execution will be aborted.");
                    return false;
                }

                if( (aux = e.attribute( "type", "" )) == "")
                    rigid = true;
                else if(aux == "rigid")
                    rigid = true;
                else if(aux == "postural")
                    rigid = false;
                else {
                    AppendToLog("ObjectModelReader: Error reading XML information. Attribute 'type' with unrecognized value. Allowed values are 'rigid' and 'postural'. Execution will be aborted.");
                    return false;
                }

                model->IsRigid = rigid;

                if( (omodel = e.attribute( "model", "" )) == "") {
                    AppendToLog("ObjectModelReader: Error reading XML information. Undefined attribute 'model' for ObjectModel in file '" + fileName + "'. Execution will be aborted.");
                    return false;
                }
                if(omodel == "ParallelpipedModel") {
                    if(!readParallelpipedModel(model, n))
                        return false;
                } else {
                    AppendToLog("ObjectModelReader: Error reading XML information. Parameter 'model' value '" + omodel + "' does not define an available object model in file '" + fileName + "'. Valid models are: ParallelpipedModel. Execution will be aborted.");
                    return false;
                }

                //Sets a model:
                objectModels[type] = model;
            } else {
                AppendToLog("ObjectModelReader: Error reading XML information. Element should name an ObjectModel in file '" + fileName + "'. Execution will be aborted.");
                return false;
            }
        } else {
            AppendToLog("ObjectModelReader: Error reading XML information. Node to element transform failed in file '" + fileName + "'. Execution will be aborted.");
            return false;
        }

        n = n.nextSibling();
    }

    return true;
}

bool ObjectModelReader::readParallelpipedModel(SpModelInterface model, QDomNode& n) {
    QDomElement e;
    QString aux;
    QDomNode m, v;
    std::vector<QString> names;
    names.push_back("width");
    names.push_back("height");
    names.push_back("depth");
    names.push_back("velocity");

    std::vector<QString>::iterator it, it_end = names.end();
    model->velocity_defined = false;
    for(it = names.begin(); it != it_end; it++) {
        if( ( m = XmlCommon::getParameterNode(*it, n) ).isNull() ) {
            if(*it != "velocity") { //It can be undefined, as a default is built
                AppendToLog("ObjectModelReader: Error reading ParallelpipedModel. Attribute model parameter " + *it + " must be defined for a ParallelpipedModel in file '" + m_fileName + "'. Execution will be aborted.");
                return false;
            }
        } else {
            e = m.toElement();
            if( !e.isNull() ) {
                if( (aux = e.attribute( "model", "" )) != "GaussianFunction" ) {
                    AppendToLog("ObjectModelReader: Error reading ParallelpipedModel. Parameter 'model' value '" + aux + "' does not define an available attribute model in file '" + m_fileName + "'. Valid models are: ParallelpipedModel. Execution will be aborted.");
                    return false;
                }
            } else  {
                AppendToLog("ObjectModelReader: ParallelpipedModel: Error reading XML information. Node to element transform failed in file '" + m_fileName + "'. Execution will be aborted.");
                return false;
            }
            SpGaussianFunction g(new GaussianFunction());
            g->setParameters(m);
            (model->m_mapCriteria)[*it].score = 0.0;
            model->m_mapCriteria[*it].reliability = 0.0;
            model->m_mapCriteria[*it].spFunction = g;
            if(*it == "velocity")
                model->velocity_defined = true;
        }
    }

    if(!model->velocity_defined) {
        double max = m_data->sceneModel->m_MaximalObjectSpeed;        
        SpGaussianFunction g(new GaussianFunction());
        g->mean = max/2.0;
        g->sigma = max*2.0;
        g->min = 0.0;
        g->max = max;
        model->m_mapCriteria["velocity"].spFunction = g;
        model->m_mapCriteria["velocity"].score = 0.0;
        model->m_mapCriteria["velocity"].reliability = 0.0;
    }

    if(!model->IsRigid) {
        if( ( m = XmlCommon::getParameterNode("PostureList", n) ).isNull() ) {
            AppendToLog("ObjectModelReader: reading ParallelpipedModel: warning: Parameter 'PostureList' not defined for a postural object. Changing to rigid.");
            model->IsRigid = true;
        } else {
            QDomNode k = m.toElement().firstChild();
            QString omodel;
            ObjectSubtype subtype;
            if(k.isNull()) {
                AppendToLog("ObjectModelReader: reading ParallelpipedModel: warning: No parameters 'Posture' inside 'PostureList'. Changing to rigid.");
                model->IsRigid = true;
            }
            while( !k.isNull() ) {
                e = k.toElement();
                SpModelInterface submodel(new ModelInterface(m_data));
                if( !e.isNull() ) {
                    if( e.tagName() == "Posture" ) {
                        if( (aux = e.attribute( "name", "" )) == "") {
                            AppendToLog("ObjectModelReader: Error reading XML information. Undefined attribute 'name' for Posture in file '" + m_fileName + "'. Execution will be aborted.");
                            return false;
                        }
                        subtype = Blob::getSubtypeFromName(aux.toStdString());
                        if(subtype == ST_UNKNOWN) {
                            AppendToLog("ObjectModelReader: Error reading XML information. Attribute 'name' does not correspond to any pre-defined subtype. Check ObjectSubtype in file 'src/common.h' for allowed subtypes. Execution will be aborted.");
                            return false;
                        }

                        submodel->IsRigid = true;

                        if( (omodel = e.attribute( "model", "" )) == "") {
                            AppendToLog("ObjectModelReader: Error reading XML information. Undefined attribute 'model' for ObjectModel in file '" + m_fileName + "'. Execution will be aborted.");
                            return false;
                        }
                    }
                } else {
                    AppendToLog("ObjectModelReader: Parallelpiped model: Error reading XML information. Node to element transform failed in file '" + m_fileName + "'. Execution will be aborted.");
                    return false;
                }

                if(omodel == "ParallelpipedModel") {
                    if(!readParallelpipedModel(submodel, k))
                        return false;
                } else {
                    AppendToLog("ParallelpipedModel: Error reading XML information. Parameter 'model' value '" + omodel + "' does not define an available object model in file '" + m_fileName + "'. Valid models are: ParallelpipedModel. Execution will be aborted.");
                    return false;
                }

                model->addSubModel(subtype, submodel);
                k = k.nextSibling();
            }
        }
    }
    return true;
}
