#include "ReliableMobileObjectList.h"

ReliableMobileObjectList::ReliableMobileObjectList() {}

ReliableMobileObjectList::~ReliableMobileObjectList() {
    reliableMobileObjects.clear();
}

std::deque<SpReliableMobileObject>::iterator ReliableMobileObjectList::begin() {
    return reliableMobileObjects.begin();
}

std::deque<SpReliableMobileObject>::iterator ReliableMobileObjectList::end() {
    return reliableMobileObjects.end();
}

SpReliableMobileObject ReliableMobileObjectList::operator[](int index){
    return reliableMobileObjects[index];
}

unsigned int ReliableMobileObjectList::size() {
    return reliableMobileObjects.size();
}

bool ReliableMobileObjectList::empty() {
    return reliableMobileObjects.empty();
}

void ReliableMobileObjectList::insert(SpReliableMobileObject rMobileObject) {
    reliableMobileObjects.push_back(rMobileObject);
}

void ReliableMobileObjectList::insert(std::deque<SpReliableMobileObject>::iterator pos, std::deque<SpReliableMobileObject>::iterator f, std::deque<SpReliableMobileObject>::iterator l) {
    reliableMobileObjects.insert(pos, f, l);
}

std::deque<SpReliableMobileObject>::iterator ReliableMobileObjectList::erase(std::deque<SpReliableMobileObject>::iterator pos) {
    return reliableMobileObjects.erase(pos);
}


void ReliableMobileObjectList::clear() {
    reliableMobileObjects.clear();
}

std::ostream& operator<<(std::ostream& out, ReliableMobileObjectList& rmol){
    std::deque<SpReliableMobileObject>::iterator it;
    int i = 0;
    for(it = rmol.begin(); it != rmol.end(); it++, i++)
        out << "Reliable Mobile Object "<< i <<" :" << std::endl << (*it) << std::endl;
    return out;
}
