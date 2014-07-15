#ifndef OBJECT_H
#define OBJECT_H
#include <deque>
#include<QString>
#include "blob.h"

//Informacion 3D:
class Info3D {
    public:
        Info3D(){}
        ~Info3D(){}
        int x; //Coordenada x 3D del objeto
        int y; //Coordenada y 3D del objeto
        int Vx; //Velocidad componente x 3D del objeto
        int Vy; //Velocidad componente y 3D del objeto
};

class Object {
    public:
        int label; //Etiqueta de objeto (unica por objeto en la secuencia).
        Blob info2D; //Informacion 2D estimada DEL OBJETO.
        ObjectType type; //Etiqueta de tipo (PERSON, VEHICLE, UNKNOWN) (si es calculada).
        std::deque<Info3D> trajectory; //Trayectoria del objeto.
        static ObjectType extractType(QString type);
        static QString getTypeString(ObjectType type);
};


#endif // OBJECT_H
