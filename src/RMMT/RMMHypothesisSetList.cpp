#include "RMMHypothesisSetList.h"

RMMHypothesisSetList::RMMHypothesisSetList() {}

RMMHypothesisSetList::~RMMHypothesisSetList() {
    RMMHypothesisSets.clear();
}

std::deque<SpRMMHypothesisSet>::iterator RMMHypothesisSetList::begin() {
    return RMMHypothesisSets.begin();
}

std::deque<SpRMMHypothesisSet>::iterator RMMHypothesisSetList::end() {
    return RMMHypothesisSets.end();
}

SpRMMHypothesisSet RMMHypothesisSetList::operator[](int index){
    return RMMHypothesisSets[index];
}

unsigned int RMMHypothesisSetList::size() {
    return RMMHypothesisSets.size();
}

bool RMMHypothesisSetList::empty() {
    return RMMHypothesisSets.empty();
}

void RMMHypothesisSetList::insert(SpRMMHypothesisSet hset) {
    RMMHypothesisSets.push_back(hset);
}

void RMMHypothesisSetList::insert(std::deque<SpRMMHypothesisSet>::iterator pos, std::deque<SpRMMHypothesisSet>::iterator f, std::deque<SpRMMHypothesisSet>::iterator l) {
    RMMHypothesisSets.insert(pos, f, l);
}

std::deque<SpRMMHypothesisSet>::iterator RMMHypothesisSetList::erase(std::deque<SpRMMHypothesisSet>::iterator pos) {
    return RMMHypothesisSets.erase(pos);
}


void RMMHypothesisSetList::clear() {
    RMMHypothesisSets.clear();
}

std::ostream& operator<<(std::ostream& out, RMMHypothesisSetList& hsl){
    std::deque<SpRMMHypothesisSet>::iterator it;
    int i = 0;
    for(it = hsl.begin(); it != hsl.end(); it++, i++)
        out << "Hypothesis set "<< i <<" :" << std::endl << (*it) << std::endl;
    return out;
}
