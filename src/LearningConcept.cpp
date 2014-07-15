#include "LearningConcept.h"
#include "MathFunctions.h"
#include <list>
#include <iostream>

namespace miles {

  NumericalAttributeValuePair::NumericalAttributeValuePair(attribute_id i_id, double mean, double sigma, double rsum) {
    SpGaussianFunction aux_dist(new GaussianFunction(mean, sigma));
    setId(i_id);    
    RSum = rsum;
    distribution = aux_dist;
  }


  NumericalAttributeValuePair::~NumericalAttributeValuePair() {  }

  void NumericalAttributeValuePair::setId(attribute_id i_id) {
    id = i_id;
  }

  attribute_id NumericalAttributeValuePair::getId() {
    return id;
  }


  void NumericalAttributeValuePair::modifyValue(double mean, double sigma) {
    distribution->setMean(mean);
    distribution->setSigma(sigma);    
  }

  double NumericalAttributeValuePair::getP() {
    return p;
  }

  void NumericalAttributeValuePair::setP(double i_p) {
    p=i_p;
  }

  double NumericalAttributeValuePair::getMean() {
    return distribution->mean;
  }

  double NumericalAttributeValuePair::getSigma() {
    return distribution->sigma;
  }

  double NumericalAttributeValuePair::getRSum() {
    return RSum;
  }

  void NumericalAttributeValuePair::setMean(double mean) {
    distribution->setMean(mean);
  }

  void NumericalAttributeValuePair::setSigma(double sigma) {
    distribution->setSigma(sigma);
  }

  void NumericalAttributeValuePair::setRSum(double i_RSum) {
    RSum = i_RSum;
  }

  
  void NumericalAttributeValuePair::print(SpLearningAttributes availableAttributes) {
    std::cout << "\t\tLabel: " << availableAttributes->nameFromId[id] << "\n";
    std::cout.flush();
    std::cout << "\t\tProbability: " << p << "\n";
    std::cout.flush();
    std::cout << "\t\tMean: " << distribution->mean << "\n";
    std::cout.flush();
    std::cout << "\t\tSigma: " << distribution->sigma << "\n";
    std::cout.flush();
    std::cout << "\t\tRSum: " << RSum << "\n";
    std::cout.flush();
  }

  bool NumericalAttributeValuePair::operator==(NumericalAttributeValuePair avp) {
    if(getId() == avp.getId())
      return true;

    return false;
  }
 
  //Symbolic Implementations
  SymbolicValue::SymbolicValue(): RSum(0.0), P(0.0), N(0) {}

  SymbolicValue::SymbolicValue(double rsum, double p, int n): RSum(rsum), P(p), N(n) {}

  SymbolicValue::~SymbolicValue() {}

  SymbolicAttributeValuePair::SymbolicAttributeValuePair(attribute_id i_id, double rsum, std::map<std::string, SpSymbolicValue> *values): id(i_id), RSum(rsum) {
    int i, n = values->size();
    SpSymbolicValue currentValue;
    std::map<std::string, SpSymbolicValue>::iterator valIt = values->begin();
    
    for(i=0; i<n; i++, valIt++) {
      currentValue = (*valIt).second;
      SpSymbolicValue newValue(new SymbolicValue(currentValue->RSum, currentValue->P, currentValue->N));
      this->values[(*valIt).first] = newValue;
    }
  }

  SymbolicAttributeValuePair::SymbolicAttributeValuePair(attribute_id i_id, double reliability, std::string initial_value, std::set<std::string> *values): id(i_id), RSum(reliability) {
    int i, n = values->size();
    std::string currentValue;
    std::set<std::string>::iterator valIt = values->begin();

    for(i=0; i<n; i++, valIt++) {
      currentValue = (*valIt);
      if(currentValue == initial_value) {
	SpSymbolicValue newValue(new SymbolicValue(reliability, 1.0, 1));
	this->values[currentValue] = newValue;
      } else {
	SpSymbolicValue newValue(new SymbolicValue(0.0, 0.0, 0));
	this->values[currentValue] = newValue;
      }
    }
  }

  SymbolicAttributeValuePair::~SymbolicAttributeValuePair() {  }

  void SymbolicAttributeValuePair::setId(attribute_id i_id) {
    id = i_id;
  }

  attribute_id SymbolicAttributeValuePair::getId() {
    return id;
  }


  SpSymbolicValue SymbolicAttributeValuePair::modifyValue(std::string value, double R) {
    SpSymbolicValue rvalue = (*values.find(value)).second;
    rvalue->RSum += R;
    rvalue->N++;
    rvalue->P = rvalue->RSum / RSum; 

    return rvalue;
  }

  SpSymbolicValue SymbolicAttributeValuePair::getValueInfo(std::string value) {
    return (*values.find(value)).second;
  }

  double SymbolicAttributeValuePair::getValueRSum(std::string value) {
    return ((*values.find(value)).second)->RSum;
  }

  double SymbolicAttributeValuePair::getValueP(std::string value) {
    return ((*values.find(value)).second)->P;
  }

  int SymbolicAttributeValuePair::getValueN(std::string value) {
    return ((*values.find(value)).second)->N;
  }

  double SymbolicAttributeValuePair::getRSum() {
    return RSum;
  }

  void SymbolicAttributeValuePair::setRSum(double i_RSum) {
    RSum = i_RSum;
  }

  double SymbolicAttributeValuePair::getP() {
    return p;
  }

  void SymbolicAttributeValuePair::setP(double i_p) {
    p=i_p;
  }
  
  void SymbolicAttributeValuePair::print(SpLearningAttributes availableAttributes) {
    std::cout << "\t\tLabel: " << availableAttributes->nameFromId[id] << "\n";
    std::cout.flush();
    std::cout << "\t\tProbability: " << p << "\n";
    std::cout.flush();
    std::cout << "\t\tRSum: " << RSum << "\n";
    std::cout.flush();
    std::cout << "\t\tValues:\n";
    std::cout.flush();

    std::map<std::string, SpSymbolicValue>::iterator valuesIt = values.begin();
    int i, n = values.size();
    SpSymbolicValue currentValue;
    for(i=0; i<n; i++, valuesIt++) {
      currentValue = (*valuesIt).second;
      std::cout << "\t\t\t" << (*valuesIt).first 
		<< "\tRSum = " << currentValue->RSum
		<< "\tP = " << currentValue->P
		<< "\tN = " << currentValue->N;
    }
  }

  bool SymbolicAttributeValuePair::operator==(SymbolicAttributeValuePair avp) {
    if(getId() == avp.getId())
      return true;

    return false;
  }


  //Concept Implementations
  LearningStateConcept::LearningStateConcept() : N(0), NE(0), P(0.0), score(0) {
  }

  LearningStateConcept::LearningStateConcept(double i_P) : N(0), NE(0), P(i_P), score(0) {
  }

  LearningStateConcept::~LearningStateConcept() {
    numericalAttributeList.clear();
    symbolicAttributeList.clear();
  }

  SpNumericalAttributeValuePair LearningStateConcept::insertNumericalPair(attribute_id id, double mean, double sigma, double rsum) {
    SpNumericalAttributeValuePair new_pair(new NumericalAttributeValuePair(id, mean, sigma, rsum));
    numericalAttributeList[id] = new_pair;
    return new_pair;
  }

  SpSymbolicAttributeValuePair LearningStateConcept::insertSymbolicPair(attribute_id id, double rsum, std::map<std::string, SpSymbolicValue> *values) {
    SpSymbolicAttributeValuePair new_pair (new SymbolicAttributeValuePair(id, rsum, values));
    symbolicAttributeList[id] = new_pair;
    return new_pair;
  }

  SpSymbolicAttributeValuePair LearningStateConcept::insertSymbolicPair(attribute_id id, double reliability, std::string initial_value, std::set<std::string> *values) {
    SpSymbolicAttributeValuePair new_pair (new SymbolicAttributeValuePair(id, reliability, initial_value, values));
    symbolicAttributeList[id] = new_pair;
    return new_pair;
  }

  
  bool LearningStateConcept::numericalIdExists(attribute_id id) {
    if(numericalAttributeList.count(id) > 0)
      return true;
    return false;
  }

  bool LearningStateConcept::symbolicIdExists(attribute_id id) {
    if(symbolicAttributeList.count(id) > 0)
      return true;
    return false;
  }

  void LearningStateConcept::removeNumericalPair(attribute_id id) {
    numericalAttributeList.erase(id);
  }

  void LearningStateConcept::removeSymbolicPair(attribute_id id) {
    symbolicAttributeList.erase(id);
  }
  
  SpNumericalAttributeValuePair LearningStateConcept::getNumericPair(attribute_id id) {
    return (*numericalAttributeList.find(id)).second;
  }

  SpSymbolicAttributeValuePair LearningStateConcept::getSymbolicPair(attribute_id id) {
    return (*symbolicAttributeList.find(id)).second;
  }

  int LearningStateConcept::numericalPairsListSize() {
    return numericalAttributeList.size();
  }

  int LearningStateConcept::symbolicPairsListSize() {
    return symbolicAttributeList.size();
  }

  double LearningStateConcept::getP() {
    return P;
  }
  
  void LearningStateConcept::setP(double i_P) {
    P=i_P;
  }
  
  int LearningStateConcept::getN() {
    return N;
  }
  
  void LearningStateConcept::setN(int number) {
    N=number;
  }

  long int LearningStateConcept::getID() {
    return id;
  }

  void LearningStateConcept::setID(long int i_id) {
    id=i_id;
  }
  int LearningStateConcept::getParent(){
    return parent;
  }

  void LearningStateConcept::setParent(int parentID){
    parent=parentID;
  }

  double LearningStateConcept::getScore() {
    return score;
  }

  void LearningStateConcept::setScore(double i_score) {
    score=i_score;
  }

  double LearningStateConcept::getDiff() {
    return diff;
  }

  void LearningStateConcept::setDiff(double i_diff) {
    diff=i_diff;
  }

void LearningStateConcept::setConceptBorderToArray(double *Ps, int w, int h, attribute_id _x, attribute_id _y, double maxP, double ignorePThreshold) {
    SpNumericalAttributeValuePair x = numericalAttributeList[_x],
                                  y = numericalAttributeList[_y];
    double sigmax = x->getSigma(), sigmay = y->getSigma(),
           the_log = log((ignorePThreshold*maxP*sigmax*sigmay*2*M_PI)/(getP()));
    int __x, diffx = (int)round(sqrt(fabs(2*sigmax*sigmax*the_log))),
        __y, diffy = (int)round(sqrt(fabs(2*sigmay*sigmay*the_log)));

    double Px, Py;
    int i, j;

    Px = MathFunctions::ONE_DIV_SQRT_2PI/sigmax;
    Py = MathFunctions::ONE_DIV_SQRT_2PI/sigmay;

    double meanx = x->getMean(), newP,
           meany = y->getMean(), Paux = 255*P/maxP;
    newP = Paux*Px*Py;
    int diffx2 = diffx - 5, diffy2 = diffy - 5;
    for(i=-diffx+1; i<diffx; i++) {
        __x = (int)meanx + i;
        if(__x >= 0 && __x < w) {
            for(j=-diffy+1; j<diffy; j++) {
                __y = (int)meany + j;
                if(__y >= 0 && __y < h) {
                    if(diffy < 1 || diffx < 1) {
                        if(i*i/((double)(diffx*diffx)) + j*j/((double)(diffy*diffy)) <= 1.0) { //Just process inside the ellipse
                            if(newP > Ps[__y*w + __x])
                                Ps[__y*w + __x] = newP;
                        }
                    } else {
                        if(    i*i/((double)(diffx*diffx)) + j*j/((double)(diffy*diffy)) <= 1.0
                            && i*i/((double)(diffx2*diffx2)) + j*j/((double)(diffy2*diffy2)) >= 1.0) { //Just process inside the ellipse
                            if(newP > Ps[__y*w + __x])
                                Ps[__y*w + __x] = newP;
                        }
                    }
                }
            }
        }
    }
}

void LearningStateConcept::setConceptToArray(double *Ps, int w, int h, attribute_id _x, attribute_id _y, double maxP, double ignorePThreshold) {
    SpNumericalAttributeValuePair x = numericalAttributeList[_x],
                                  y = numericalAttributeList[_y];
    double sigmax = x->getSigma(), sigmay = y->getSigma(),
           the_log = log((ignorePThreshold*maxP*sigmax*sigmay*2*M_PI)/(getP()));
    int __x, diffx = (int)round(sqrt(fabs(2*sigmax*sigmax*the_log))),
        __y, diffy = (int)round(sqrt(fabs(2*sigmay*sigmay*the_log)));

    double *Px = new double[diffx], *Py = new double[diffy];
    int i, j;

//    std::cout << "State: " << id << "; Parent: " << parent << std::endl;
    //std::cout << "x: " << x->getMean() << "; sigmax: " << sigmax << std::endl;
    //std::cout << "Px:" << std::endl;
    for(i=0; i<diffx; i++) {
        Px[i] = MathFunctions::ONE_DIV_SQRT_2PI*exp(-(i*i)/(2*sigmax*sigmax))/sigmax;
        //std::cout << "\t Py[: " << i << "] = " << MathFunctions::ONE_DIV_SQRT_2PI*255*Px[i]/(maxP*sigmay) << std::endl;
    }

    //std::cout << "y: " << y->getMean() << "; sigmay: " << sigmay << std::endl;
    //std::cout << "Py:" << std::endl;
    for(i=0; i<diffy; i++) {
        Py[i] = MathFunctions::ONE_DIV_SQRT_2PI*exp(-(i*i)/(2*sigmay*sigmay))/sigmay;
        //std::cout << "\t Py[: " << i << "] = " << MathFunctions::ONE_DIV_SQRT_2PI*255*Py[i]/(maxP*sigmax) << std::endl;
    }
    double meanx = x->getMean(), newP,
           meany = y->getMean(), Paux = 255*P/maxP;
    for(i=-diffx+1; i<diffx; i++) {
        __x = (int)meanx + i;
        if(__x >= 0 && __x < w) {
            for(j=-diffy+1; j<diffy; j++) {
                __y = (int)meany + j;
                if(__y >= 0 && __y < h) {
                    if(i*i/((double)(diffx*diffx)) + j*j/((double)(diffy*diffy)) <= 1.0) { //Just process inside the ellipse
                        newP = Paux*Px[abs(i)]*Py[abs(j)];
                        //std::cout << "\t newP(" << __x << "," << __y << ") = " << newP
                        //          << "(ellipse test(" << i << "," << j << "): " << i*i/(diffx*diffx) + j*j/(diffy*diffy) <<  ")" << std::endl;
                        if(newP > Ps[__y*w + __x])
                            Ps[__y*w + __x] = newP;
                    }
                }
            }
        }
    }
    delete[] Px;
    delete[] Py;
}

void LearningStateConcept::setFlatConceptToArray(double *Ps, int w, int h, attribute_id _x, attribute_id _y, double maxP, double ignorePThreshold) {
    SpNumericalAttributeValuePair x = numericalAttributeList[_x],
                                  y = numericalAttributeList[_y];
    double sigmax = x->getSigma(), sigmay = y->getSigma(),
           the_log = log((ignorePThreshold*maxP*sigmax*sigmay*2*M_PI)/(getP()));
    int __x, diffx = (int)round(sqrt(fabs(2*sigmax*sigmax*the_log))),
        __y, diffy = (int)round(sqrt(fabs(2*sigmay*sigmay*the_log)));

    double Px, Py;
    int i, j;

    Px = MathFunctions::ONE_DIV_SQRT_2PI/sigmax;
    Py = MathFunctions::ONE_DIV_SQRT_2PI/sigmay;

    double meanx = x->getMean(), newP,
           meany = y->getMean(), Paux = 255*P/maxP;
    newP = Paux*Px*Py;
    for(i=-diffx+1; i<diffx; i++) {
        __x = (int)meanx + i;
        if(__x >= 0 && __x < w) {
            for(j=-diffy+1; j<diffy; j++) {
                __y = (int)meany + j;
                if(__y >= 0 && __y < h) {
                    if(i*i/((double)(diffx*diffx)) + j*j/((double)(diffy*diffy)) <= 1.0) { //Just process inside the ellipse
                        if(newP > Ps[__y*w + __x])
                            Ps[__y*w + __x] = newP;
                    }
                }
            }
        }
    }
}

void LearningStateConcept::setAttributeConceptToArray(double *As, int w, int h, attribute_id _x, attribute_id _y, attribute_id _a, double maxA, double ignorePThreshold) {
    SpNumericalAttributeValuePair x = numericalAttributeList[_x],
                                  y = numericalAttributeList[_y],
                                  a = numericalAttributeList[_a];
    double sigmax = x->getSigma(), sigmay = y->getSigma();
    int __x, diffx = (int)round(sigmax),
        __y, diffy = (int)round(sigmay);

    int i, j;
    double meanx = x->getMean(),
           meany = y->getMean(), Aaux = 255*a->getMean()/maxA;
    for(i=-diffx+1; i<diffx; i++) {
        __x = (int)meanx + i;
        if(__x >= 0 && __x < w) {
            for(j=-diffy+1; j<diffy; j++) {
                __y = (int)meany + j;
                if(__y >= 0 && __y < h) {
                    if(i*i/((double)(diffx*diffx)) + j*j/((double)(diffy*diffy)) <= 1.0) { //Just process inside the ellipse
                        if(Aaux > As[__y*w + __x])
                            As[__y*w + __x] = Aaux;
                    }
                }
            }
        }
    }
}

void LearningStateConcept::print(SpLearningAttributes availableAttributes) {
    int i, n;

    std::cout << "\nObject Probability:\t" << P << "\nAttributes:\n";
    std::cout.flush();

    std::set<SpNumericalAttributeValuePair>::iterator niter;
    std::set<SpSymbolicAttributeValuePair>::iterator siter;
    std::cout << "Numerical Attibutes:\n";
    n = numericalAttributeList.size();
    std::map <attribute_id, SpNumericalAttributeValuePair>::iterator iter;
    for (i=0, iter=numericalAttributeList.begin(); iter != numericalAttributeList.end(); i++, iter++) {
      std::cout.flush();
      (*iter).second->print(availableAttributes);
    }
    //TODO SYMBOLIC
  }


  double LearningStateConcept::getPositionP(attribute_id _x, attribute_id _y) {
    if(numericalAttributeList.count(_x) == 0)
        return 0.0;
    if(numericalAttributeList.count(_y) == 0)
        return 0.0;
    double p = P*MathFunctions::ONE_DIV_SQRT_2PI/numericalAttributeList[_x]->getSigma()*MathFunctions::ONE_DIV_SQRT_2PI/numericalAttributeList[_y]->getSigma();
    return p;
  }


  LearningEventConcept::LearningEventConcept(long int i_id, double mean, double sigma) {
    SpGaussianFunction aux_dist(new GaussianFunction(mean, sigma));
    id = i_id;    
    N = 1;
    timeStayingAtOutgoingState = aux_dist;
  }

  LearningEventConcept::~LearningEventConcept() {}

  double LearningEventConcept::getMean() {
    return timeStayingAtOutgoingState->mean;
  }

  double LearningEventConcept::getSigma() {
    return timeStayingAtOutgoingState->sigma;
  }

  void LearningEventConcept::setMean(double mean) {
    timeStayingAtOutgoingState->setMean(mean);
  }

  void LearningEventConcept::setSigma(double sigma) {
    timeStayingAtOutgoingState->setSigma(sigma);
  }




} // end namespace miles

