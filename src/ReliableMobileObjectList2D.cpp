#include "ReliableMobileObjectList2D.h"

ReliableMobileObjectList2D::ReliableMobileObjectList2D() {}

ReliableMobileObjectList2D::~ReliableMobileObjectList2D() {
    reliableMobileObjects.clear();
}

std::deque<SpReliableMobileObject2D>::iterator ReliableMobileObjectList2D::begin() {
    return reliableMobileObjects.begin();
}

std::deque<SpReliableMobileObject2D>::iterator ReliableMobileObjectList2D::end() {
    return reliableMobileObjects.end();
}

SpReliableMobileObject2D ReliableMobileObjectList2D::operator[](int index){
    return reliableMobileObjects[index];
}

unsigned int ReliableMobileObjectList2D::size() {
    return reliableMobileObjects.size();
}

bool ReliableMobileObjectList2D::empty() {
    return reliableMobileObjects.empty();
}

void ReliableMobileObjectList2D::insert(SpReliableMobileObject2D rMobileObject) {
    reliableMobileObjects.push_back(rMobileObject);
}

void ReliableMobileObjectList2D::insert(std::deque<SpReliableMobileObject2D>::iterator pos, std::deque<SpReliableMobileObject2D>::iterator f, std::deque<SpReliableMobileObject2D>::iterator l) {
    reliableMobileObjects.insert(pos, f, l);
}

std::deque<SpReliableMobileObject2D>::iterator ReliableMobileObjectList2D::erase(std::deque<SpReliableMobileObject2D>::iterator pos) {
    return reliableMobileObjects.erase(pos);
}


void ReliableMobileObjectList2D::clear() {
    reliableMobileObjects.clear();
}

std::ostream& operator<<(std::ostream& out, ReliableMobileObjectList2D& rmol){
    std::deque<SpReliableMobileObject2D>::iterator it;
    int i = 0;
    for(it = rmol.begin(); it != rmol.end(); it++, i++)
        out << "Reliable Mobile Object "<< i <<" :" << std::endl << (*it) << std::endl;
    return out;
}
