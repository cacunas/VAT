#include "object.h"

ObjectType Object::extractType(QString type) {
    if(type == "PERSON")
        return PERSON;
    if(type == "VEHICLE")
        return VEHICLE;
    return UNKNOWN;
}

QString Object::getTypeString(ObjectType type) {
    if(type == PERSON)
        return "PERSON";
    if(type == VEHICLE)
        return "VEHICLE";
    return "UNKNOWN";
}
