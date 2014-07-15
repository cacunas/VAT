#include "LearningAttributesSpecification.h"
#include "Datapool.h"
#include "MODELS/Blob2DFromBGSubstractionModel.h"

#define MIN_RELIABILITY 0.000001

//Implementation for attribute functions defined in file CLearningAttributesSpecification.h
namespace miles {

//For symbolic attributes insert the definition of the static variable containing the list of
//valid attribute values
  std::set<std::string> SKITCHEN::values;
  std::set<std::string> STABLE::values;
  std::set<std::string> SSOFA::values;
  std::set<std::string> DEGU_CORNER::values;
  bool DEGU_CORNER::checked = false;
  bool DEGU_CORNER::valid = false;
  std::map< std::string, QSharedPointer<world::ZoneH> > DEGU_CORNER::zones;

  //Function implementations here:

  //For numerical attributes:

  //Homography Attributes
  double HX::getValue(SpMObject mobile, Datapool& datapool) {
      SpReliabilitySingleModelInterface m = mobile->getSubModel("Blob2DFromBGSubstractionModel");
      if(m.isNull())
          return 0;
      return m->dynamics.dynamics["X"].att.value;
      //double X,Y;
      /*if(datapool.sceneModel->hmatrix_filled) {
        datapool.sceneModel->imgToHomographyCoords(datapool.sceneModel->h_matrix,
                                                   nearbyint(mobile->t2DSpatialData.X),
                                                   nearbyint(mobile->t2DSpatialData.Y),
                                                   &X, &Y);
      }*/
  }
  double HX::getReliability(SpMObject mobile, Datapool& datapool) {
      SpReliabilitySingleModelInterface m = mobile->getSubModel("Blob2DFromBGSubstractionModel");
      if(m.isNull())
          return 0;
      return m->dynamics.dynamics["X"].att.VR.RC;
  }

  double HY::getValue(SpMObject  mobile, Datapool& datapool) {
      SpReliabilitySingleModelInterface m = mobile->getSubModel("Blob2DFromBGSubstractionModel");
      if(m.isNull())
          return 0;
      return m->dynamics.dynamics["Y"].att.value;
      //double X,Y;
      /*if(datapool.sceneModel->hmatrix_filled) {
        datapool.sceneModel->imgToHomographyCoords(datapool.sceneModel->h_matrix,
                                                   nearbyint(mobile->t2DSpatialData.X),
                                                   nearbyint(mobile->t2DSpatialData.Y),
                                                     &X, &Y);
      }*/
  }

  double HY::getReliability(SpMObject  mobile, Datapool& datapool) {
      SpReliabilitySingleModelInterface m = mobile->getSubModel("Blob2DFromBGSubstractionModel");
      if(m.isNull())
          return 0;
      return m->dynamics.dynamics["Y"].att.VR.RC;
  }

  //2D Spatial Attributes
  double X::getValue(SpMObject mobile, Datapool& datapool) {
      SpReliabilitySingleModelInterface m = mobile->getSubModel("Blob2DFromBGSubstractionModel");
      if(m.isNull())
          return 0;
      return m->dynamics.dynamics["X"].att.value;
  }
  double X::getReliability(SpMObject mobile, Datapool& datapool) {
      SpReliabilitySingleModelInterface m = mobile->getSubModel("Blob2DFromBGSubstractionModel");
      if(m.isNull())
          return 0;
      return m->dynamics.dynamics["X"].att.VR.RC;
  }

  double Y::getValue(SpMObject  mobile, Datapool& datapool) {
      SpReliabilitySingleModelInterface m = mobile->getSubModel("Blob2DFromBGSubstractionModel");
      if(m.isNull())
          return 0;
      return m->dynamics.dynamics["Y"].att.value;
  }
  double Y::getReliability(SpMObject  mobile, Datapool& datapool) {
      SpReliabilitySingleModelInterface m = mobile->getSubModel("Blob2DFromBGSubstractionModel");
      if(m.isNull())
          return 0;
      return m->dynamics.dynamics["Y"].att.VR.RC;
  }

  double VX::getValue(SpMObject  mobile, Datapool& datapool) {
      SpReliabilitySingleModelInterface m = mobile->getSubModel("Blob2DFromBGSubstractionModel");
      if(m.isNull())
          return 0;
      return m->dynamics.dynamics["X"].V.value;
  }
  double VX::getReliability(SpMObject mobile, Datapool& datapool) {
      SpReliabilitySingleModelInterface m = mobile->getSubModel("Blob2DFromBGSubstractionModel");
      if(m.isNull())
          return 0;
      return m->dynamics.dynamics["X"].V.VR.RC;
  }

  double VY::getValue(SpMObject mobile, Datapool& datapool) {
      SpReliabilitySingleModelInterface m = mobile->getSubModel("Blob2DFromBGSubstractionModel");
      if(m.isNull())
          return 0;
      return m->dynamics.dynamics["Y"].V.value;
  }
  double VY::getReliability(SpMObject  mobile, Datapool& datapool) {
      SpReliabilitySingleModelInterface m = mobile->getSubModel("Blob2DFromBGSubstractionModel");
      if(m.isNull())
          return 0;
      return m->dynamics.dynamics["Y"].V.VR.RC;
  }

  double V2D::getValue(SpMObject  mobile, Datapool& datapool) {
      SpReliabilitySingleModelInterface m = mobile->getSubModel("Blob2DFromBGSubstractionModel");
      if(m.isNull())
          return 0;

      double VX = m->dynamics.dynamics["X"].V.value,
             VY = m->dynamics.dynamics["Y"].V.value;
      return sqrt(VX*VX+VY*VY);
  }
  double V2D::getReliability(SpMObject mobile, Datapool& datapool) {
      SpReliabilitySingleModelInterface m = mobile->getSubModel("Blob2DFromBGSubstractionModel");
      if(m.isNull())
          return 0;
      return (m->dynamics.dynamics["X"].V.VR.RC + m->dynamics.dynamics["Y"].V.VR.RC)/2.0;
  }


  //2D Dimensional Attributes
  double W::getValue(SpMObject mobile, Datapool& datapool) {
      SpReliabilitySingleModelInterface m = mobile->getSubModel("Blob2DFromBGSubstractionModel");
      if(m.isNull())
          return 0;
      return m->dynamics.dynamics["W"].att.value;
  }

  double W::getReliability(SpMObject mobile, Datapool& datapool) {
      SpReliabilitySingleModelInterface m = mobile->getSubModel("Blob2DFromBGSubstractionModel");
      if(m.isNull())
          return 0;
      return m->dynamics.dynamics["W"].att.VR.RC;
  }

  double H::getValue(SpMObject mobile, Datapool& datapool) {
      SpReliabilitySingleModelInterface m = mobile->getSubModel("Blob2DFromBGSubstractionModel");
      if(m.isNull())
          return 0;
      return m->dynamics.dynamics["H"].att.value;
  }

  double H::getReliability(SpMObject mobile, Datapool& datapool) {
      SpReliabilitySingleModelInterface m = mobile->getSubModel("Blob2DFromBGSubstractionModel");
      if(m.isNull())
          return 0;
      return m->dynamics.dynamics["H"].att.VR.RC;
  }
  
  double VW::getValue(SpMObject mobile, Datapool& datapool) {
    return 0;
  }
  double VW::getReliability(SpMObject mobile, Datapool& datapool) {
    return 1;
  }

  double VH::getValue(SpMObject  mobile, Datapool& datapool) {
    return 0;
  }
  double VH::getReliability(SpMObject  mobile, Datapool& datapool) {
    return 1;
  }

  //Distance to context objects
  double DKITCHEN::getValue(SpMObject  mobile, Datapool& datapool) {
    return 0;
  }

  double DKITCHEN::getReliability(SpMObject  mobile, Datapool& datapool) {
    return 1;
  }
  
  double DTABLE::getValue(SpMObject  mobile, Datapool& datapool) {
    return 0;
  }

  double DTABLE::getReliability(SpMObject  mobile, Datapool& datapool) {
    return 1;
  }

  double DSOFA::getValue(SpMObject  mobile, Datapool& datapool) {
    return 0;
  }

  //Mean between mean position coherence and reliability on context object position (1.0)
  double DSOFA::getReliability(SpMObject  mobile, Datapool& datapool) {
    return 1;
  }

  //Specification for symbolic attributes:


  //Symbolic distance to context objects
  std::string SKITCHEN::getValue(SpMObject  mobile, Datapool& datapool) {
      return "FAR";
  }

  double SKITCHEN::getReliability(SpMObject  mobile, Datapool& datapool) {
    return 1;
  }

  std::set<std::string> *SKITCHEN::setAllowedValues() {
    values.insert("VERY_NEAR");
    values.insert("NEAR");
    values.insert("FAR");
    return &values;
  }

  std::string DEGU_CORNER::getValue(SpMObject mobile, Datapool& datapool) {
      if(checked) {
         checked = true;
         valid = true;
         //First check zones
         std::vector< QSharedPointer<world::ZoneH> > &zones = datapool.sceneModel->ZonesH;
         if (zones.empty())
             valid = false;
         else {
            int num = 0;
            std::vector< QSharedPointer<world::ZoneH> >::iterator zone_it, zone_end = zones.end();
            for(zone_it = zones.begin(); zone_it != zone_end; zone_it++ ) {
                if(values.count((*zone_it)->name.toStdString()) == 1) {
                    num++;
                    DEGU_CORNER::zones[(*zone_it)->name.toStdString()] = (*zone_it);
                }
            }
            if(num != values.size() - 1)
                valid = false;
         }
      }

      SpReliabilitySingleModelInterface m = mobile->getSubModel("Blob2DFromBGSubstractionModel");
      if(m.isNull())
          return "NONE";

      if(valid) {
          double dist, X, Y, min_dist = DBL_MAX;
          std::string nearest;
          if(datapool.sceneModel->hmatrix_filled) {
            datapool.sceneModel->imgToHomographyCoords(datapool.sceneModel->h_matrix,
                                                       nearbyint(m->dynamics.dynamics["X"].att.value),
                                                       nearbyint(m->dynamics.dynamics["Y"].att.value),
                                                       &X, &Y);
            QSharedPointer<world::ZoneH> z;
            std::map< std::string, QSharedPointer<world::ZoneH> >::iterator it, it_end = zones.end();
            for(it = zones.begin();it != it_end; it++) {
                z = (*it).second;
                if(z->pointInZone(X,Y)) {
                    dist = z->distanceToCenter(X,Y);
                    if(dist < min_dist) {
                        min_dist = dist;
                        nearest = (*it).first;
                    }
                }
            }
            if(min_dist < DBL_MAX)
                return nearest;
         }
      }
      return "NONE";
  }


  double DEGU_CORNER::getReliability(SpMObject mobile, Datapool& datapool) {
      //Add reliability measure according to minimal corner distance
      SpReliabilitySingleModelInterface m = mobile->getSubModel("Blob2DFromBGSubstractionModel");
      if(m.isNull())
          return 0;

      return (m->dynamics.dynamics["X"].att.VR.RC + m->dynamics.dynamics["Y"].att.VR.RC)/2.0;

  }

  std::set<std::string> *DEGU_CORNER::setAllowedValues() {
      values.insert("Corner1"); //Upper-left
      values.insert("Corner2"); //Upper-right
      values.insert("Corner3"); //Lower-left
      values.insert("Corner4"); //Lower-right
      values.insert("NONE"); //Not in corner
      return &values;
  }



  std::string STABLE::getValue(SpMObject  mobile, Datapool& datapool) {
      return "FAR";
  }

  double STABLE::getReliability(SpMObject  mobile, Datapool& datapool) {
    return 1;
  }

  std::set<std::string> *STABLE::setAllowedValues() {
    values.insert("VERY_NEAR");
    values.insert("NEAR");
    values.insert("FAR");
    return &values;
  }

  std::string SSOFA::getValue(SpMObject  mobile, Datapool& datapool) {
      return "FAR";
  }

  //Mean between mean position coherence and reliability on context object position (1.0)
  double SSOFA::getReliability(SpMObject  mobile, Datapool& datapool) {
    return 1;
  }

  std::set<std::string> *SSOFA::setAllowedValues() {
    values.insert("VERY_NEAR");
    values.insert("NEAR");
    values.insert("FAR");
    return &values;
  }

} // end namespace


