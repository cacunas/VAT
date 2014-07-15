#include "trackingmodule.h"
#include "image_display.h"

#include <errno.h>
#include <iostream>
#include <QMessageBox>
#include <QPainter>
#include <deque>
#include "src/object.h"
#include <string>
#include <set>
#include <algorithm>
//#include "trackingatribute.h"

//using namespace std;
trackingModule::trackingModule(Datapool *i_data): ModuleInterface(i_data) {}

trackingModule::~trackingModule() {}

bool trackingModule::setParameters(QDomNode& config){
    // Esto hay que obternerlo desde default_config.txt
    m_pos = Bottom;
    return true;
}

bool trackingModule::updateParameters(){
    return true;
}


bool trackingModule::init(){
    return true;
}


bool trackingModule::run(){

    //double Vx,Vy;
    //int id=0;
    //ObjectType type;
    //Object o;
    //std::map<int, Object>::iterator it, end_it;

    //cout << "------------------------" << endl;
    //cout << m_data->blobs.size() << endl;

    // TODO: preguntar que onda con la altura y ancho... si se
    // hace su transformacion ...
    preMerge(10);

    //cout << m_data->blobs.size() << endl;
    //cout << "++++++++++++++++++++++++" << endl;

    bool newObjectFlag = false;
    if( m_data->objects.empty() ){
        newObjectFlag = true;
        int label = 0;
        while( !m_data->blobs.empty() ){
            Blob b;
            b = m_data->blobs.front();  // Para recorrer el deque

            // Transformacion 2d/3d
            double X,Y,x,y;
            SceneModel::getXY(m_pos,X,Y,b);
            SceneModel::imgToWorldCoordsGivenHeight(m_data->sceneModel->p_matrix, X, Y, 0.0, &x, &y);

            // Creando al objeto
            Info3D i;
            i.Vx = 0;   i.Vy = 0;
            i.x = x;    i.y = y;

            Object o;
            o.info2D = b;
            o.label = label;
            o.trajectory.push_back(i);
            // TODO: poner la etiqueta
            // Se actualiza el datapool
            m_data->fake_objects[label] = o;

            label = label + 1;

        }
    }

    if( !newObjectFlag ){        
        m_data->blobs.clear(); //TODO: borrar esto...
        m_data->objects.clear();

        /*
        map<int,Object>::iterator it, end;
        end = m_data->objects.end();
        for( it=m_data->objects.begin(); it!=end; it++ ){
            Object o;
            o = it->second;

            Info3D i;
            i = o.trajectory.front();
            o.trajectory.pop_front();
            i.Vx = 1;   i.Vy = 1;
            i.x = i.x + 1;  i.y = i.y + 1;
            o.trajectory.push_front(i);
            m_data->objects[it->first] = o;
        }*/
    }

    //cout << "tamano objetos: ";
    //cout << m_data->objects.size() << endl;

    // Se obtienen los blobs de la imagen actual
    /*
    while( !newObjectFlag && !m_data->blobs.empty() ){
        Blob b;
        //int Xu,Yl,H,W;  //Variables internas del blob actual
        double X,Y,x,y; //transformacion 2d/3d

        b = m_data->blobs.front();  // Para recorrer el deque

        // Transformacion 2d/3d
        Scene::getXY(m_pos,X,Y,b);
        Scene::img_to_world_coords_given_height(m_data->sceneModel->p_matrix, X, Y, 0.0, &x, &y);

        trackingAtribute atributoX(x);        
        trackingAtribute atributoY(y);

        atributoX.refreshAtributes(x+1.0);
        cout << "---------------" << endl;
        cout << x << endl;
        cout << atributoX.calculateProbability() << endl;
        cout << "+++++++++++++++" << endl;

        m_data->blobs.pop_front();  // Para recorrer el deque

        //if( m_data->objects.empty() ){

            id = 1;
            o.info2D = b;
            o.label = id;
            //o.type = "VEHICLE";
            Info3D i;
            i.x = x; i.y = y; i.Vx = 0.0; i.Vy = 0.0;
            o.trajectory.push_back(i);
            m_data->objects[id] = o;

        //}


        if( (it = m_data->objects.find(id)) != end_it) { //Existing object
            (*it).second.info2D = b;
            Vx = x - ((*it).second.trajectory.back()).x;
            Vy = y - ((*it).second.trajectory.back()).y;
            Info3D i;
            i.x = x; i.y = y; i.Vx = Vx; i.Vy = Vy;
            (*it).second.trajectory.push_back(i);
        } else { //new object
            o.info2D = b;
            o.label = id;
            o.type = type;
            Info3D i;
            i.x = x; i.y = y; i.Vx = 0.0; i.Vy = 0.0;
            o.trajectory.push_back(i);
            m_data->objects[id] = o;
        }

    //}*/

    return true;
}


void trackingModule::preMerge(int Hmax){
    //std::sort(m_data->blobs.begin(),m_data->blobs.end(),blobSortCriterionPos);
    /*deque <Blob>::iterator it,end;
    int blobSize = m_data->blobs.size();

    for( int i=0; i<blobSize; i++ ){
        Blob b;
        b = m_data->blobs.front();
        m_data->blobs.pop_front();
        if( b.H > (Hmax-1) ){
            m_data->blobs.push_back(b);
        }

    }

    for( it=m_data->blobs.begin(); it!=m_data->blobs.end(); it++ ){
        //int h = it->H;
        Object o;
        o.info2D = m_data->blobs.front();
        o.label = label;
        m_data->objects[label] = o;
        label = label + 1;
        //cout << it->H << endl;
        //cout << "\n" << endl;
    }*/
}

bool blobSortCriterionPos (Blob b1, Blob b2) {
    return (      BLOB_YTOP(&b1) > BLOB_YTOP(&b2))
             || ( BLOB_YTOP(&b1) == BLOB_YTOP(&b2) && BLOB_XLEFT(&b1) > BLOB_XLEFT(&b2) );
}

//bool blobSortCriterionBase (Blob b1, Blob b2) {
  //  return (b1.Xb > b2.Xb) || ((b1.Xb == b2.Xb) && (b1.Yb > b2.Yb));
//}



