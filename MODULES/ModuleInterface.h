#ifndef MODULEINTERFACE_H
#define MODULEINTERFACE_H

#include <map>
#include <string>
#include <QImage>
#include "Datapool.h"
#include <QDir>
#include "string.h"
#include "xmlcommon.h"

enum ModuleType {
    ACQUISITION,
    INITIALIZATION,
    SEGMENTATION,
    TRACKING,
    ANALYSIS,
    OTHER
};


typedef std::map<std::string, std::map<std::string, std::string> >* TParameterList;

typedef struct parameter parameter;
struct parameter{
    QString name;
    QString type;
    QString value;
    std::multimap<QString, parameter> subParam;
};


class ModuleInterface {
  public:
    ModuleInterface(Datapool *i_data);
    virtual ~ModuleInterface() = 0;

    //Set module configuration parameters
    virtual bool setParameters(QDomNode &config) = 0;

    //Initialization after reading parameters
    virtual bool init() = 0;

    //Function executed at each frame
    virtual bool run() = 0;

    //update parameters at runtime.
    virtual bool updateParameters() = 0;
    void addParameter(QString name, QString value, QString type);
    void addParameterChild(QString name, QString value, QString type, parameter *parent);
    parameter* getParameter(QString name);
    parameter* getParameter(QString name, parameter *parent);
    parameter* getParameter(QString name, std::multimap<QString, parameter> subList);

    //Mapa con el nombre del parametro y el valor.
//    std::map<QString, QString> parameters;
//    //Mapa con el nombre del parametro y el tipo.
//    std::map<QString, QString> typeParameters;
    std::deque<QString> nameParameters;
    std::deque<QString> valueParameters;
    std::deque<QString> typeParameters;

    std::deque<parameter> listParameters;

    std::string name;
    ModuleType type;

  protected:
    //Reference to external datapool:
    Datapool *m_data;

};


#endif // INTERFACEMODULE_H
