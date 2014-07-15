#include "ModuleInterface.h"

ModuleInterface::ModuleInterface(Datapool *i_data): m_data(i_data) { }

ModuleInterface::~ModuleInterface() { }


//Agrega un parametro a la lista principal de un modulo (listParameters).
void ModuleInterface::addParameter(QString name, QString value, QString type){
    parameter temp;
    temp.name = name;
    temp.value = value;
    temp.type = type;
    temp.subParam.clear();
    listParameters.push_back(temp);
}

//Agrega un subparametro perteneciente a un parametro padre.
void ModuleInterface::addParameterChild(QString name, QString value, QString type, parameter *parent){
    parameter temp;
    temp.name = name;
    temp.value = value;
    temp.type = type;
    parent->subParam.insert(std::pair<QString, parameter>(name, temp));
}

//Busca un parametro especificado pro el nombre, dentro de la lista de parametros, y de las sublista de cada parametro.
parameter* ModuleInterface::getParameter(QString name, std::multimap<QString, parameter> subList){
    std::multimap<QString, parameter>::iterator it_mm, it_mm_end = subList.end();
    parameter *temp;
    if( subList.empty())
        return 0;
    for(it_mm = subList.begin(); it_mm != it_mm_end; it_mm++){
        if((*it_mm).first == name)
            return &(*it_mm).second;
        else {
            temp = getParameter(name, (*it_mm).second.subParam);
            if( temp != 0)
                return temp;
        }
    }
    return 0;
}

//busca un parametro especificado dentro del arbol completo de parametros.
parameter* ModuleInterface::getParameter(QString name){
    parameter *temp;
    std::deque<parameter>::iterator it, it_end = listParameters.end();
    for(it = listParameters.begin(); it != it_end; it++){
        if( it->name == name)
            return &(*it);
        if( it->subParam.empty())
            continue;
        temp = getParameter(name, it->subParam);
        if(temp != 0)
            return temp;
    }
    return 0;
}

//busca un parametro dentro de la sublista de un parametro padre especifico.
parameter* ModuleInterface::getParameter(QString name, parameter *parent){
    if(parent->subParam.empty())
        return 0;
    std::multimap<QString, parameter>::iterator it, it_end = parent->subParam.end();
    for( it = parent->subParam.begin(); it != it_end; it++){
        if( (*it).first == name)
            return &(*it).second;
    }
    return 0;
}
