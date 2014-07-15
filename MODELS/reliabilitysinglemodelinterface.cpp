#include "reliabilitysinglemodelinterface.h"
#include "Datapool.h"

ReliabilitySingleModelInterface::ReliabilitySingleModelInterface():
    m_mobile(NULL) {}

ReliabilitySingleModelInterface::~ReliabilitySingleModelInterface() {}

void ReliabilitySingleModelInterface::initFromXML(Datapool *i_data, int bsize, QString name) {
    m_data = i_data;
    instances.maxLength = bsize;
    this->name = name;
}

void ReliabilitySingleModelInterface::printInstances() {}

void ReliabilitySingleModelInterface::printDynamics() {}

void ReliabilitySingleModelInterface::printGeneral(std::ostream& out) {}

void ReliabilitySingleModelInterface::printGeneral() {printGeneral(std::cout);}

bool ReliabilitySingleModelInterface::draw(QImage *image) {
    return true;
}

bool ReliabilitySingleModelInterface::draw(QPainter &painter, int r, int g, int b) {
    return true;
}


//Copy general structures
void ReliabilitySingleModelInterface::copy_general_structure(SpReliabilitySingleModelInterface i) {
    m_data = i->m_data;
    active = i->active;
    name = i->name;
    model_name = i->model_name;
    P = i->P;
    R_forward = i->R_forward;
    R_backward = i->R_backward;
    binterface_forward = i->binterface_forward;
    binterface_backward = i->binterface_backward;
    instances = i->instances;
    dynamics = i->dynamics;
    instanceTemplate = i->instanceTemplate;
    dependences = i->dependences;
    dependants = i->dependants;
    copy_structure(i);
}

void ReliabilitySingleModelInterface::copy_structure(SpReliabilitySingleModelInterface i) {}


void ReliabilitySingleModelInterface::general_copy(SpReliabilitySingleModelInterface i) {
    m_data = i->m_data;
    active = i->active;
    name = i->name;
    model_name = i->model_name;
    P = i->P;
    R_forward = i->R_forward;
    R_backward = i->R_backward;
    binterface_forward = i->binterface_forward;
    binterface_backward = i->binterface_backward;
    instances = i->instances;
    dynamics = i->dynamics;
    dependences = i->dependences;
    dependants = i->dependants;
    copy(i);
}

//Default behavior of copy is not copying anything else
void ReliabilitySingleModelInterface::copy(SpReliabilitySingleModelInterface i) { }


ModelBuffer::ModelBuffer(){ }

ModelBuffer::~ModelBuffer(){
    instances.clear();
}

std::deque<ModelInstanceDataPack>::iterator ModelBuffer::begin() {
    return instances.begin();
}

std::deque<ModelInstanceDataPack>::iterator ModelBuffer::end() {
    return instances.end();
}

std::deque<ModelInstanceDataPack>::reverse_iterator ModelBuffer::rbegin() {
    return instances.rbegin();
}

std::deque<ModelInstanceDataPack>::reverse_iterator ModelBuffer::rend() {
    return instances.rend();
}

ModelInstanceDataPack &ModelBuffer::operator[](int index){
    return instances[index];
}

ModelInstanceDataPack &ModelBuffer::back() {
    return instances.back();
}

ModelInstanceDataPack &ModelBuffer::front() {
    return instances.front();
}


unsigned int ModelBuffer::size() {
    return instances.size();
}

void ModelBuffer::insert(ModelInstanceDataPack &instance) {
    if(size() == maxLength)
        instances.pop_back();
    instances.push_front(instance);
}


void ModelBuffer::clear() {
    instances.clear();
}

void ModelBuffer::resetMaxLength(int i_maxLength) {
    maxLength = (i_maxLength <= 0) ? 1: i_maxLength;
}

void ModelBuffer::copyModels(ModelBuffer *buffer) {
    instances = buffer->instances;
}



void ReliabilitySingleModelInterface::setTrajectory2D(double &x, double &y, double &r) {
    Rectangle<int> &rec = binterface_backward.bbox;
    x = rec.xleft + rec.width/2;
    y = rec.ytop + rec.height/2;
    r = P*R_backward;
}

Rectangle<int> ReliabilitySingleModelInterface::getEstimator(double &R) {
    R = P; //considered reliability based in coherence
    return binterface_backward.bbox;
}

double ReliabilitySingleModelInterface::normalisedSigmaCoherenceReliability(double sigma, double acuity) {
#ifdef __RMM_NO_RELIABILITY_CODE__
    return 1.0;
#else
    return  sigma<=acuity ? 1.0 : acuity/sigma;
#endif
}

std::ostream& operator<<(std::ostream& out, ModelBuffer& mmb) {
    std::deque<ModelInstanceDataPack> &instances = mmb.instances;
    std::deque<ModelInstanceDataPack>::iterator it;
    int i = 0;
    for(it = instances.begin(); it != instances.end(); it++, i++){
        out << "\t\t\t\tModel " << i;
//        (*it)->printGeneral(out);
        out << std::endl;
    }
    return out;
}
