#include <map>
#include <deque>
#include "geometric.h"

#ifndef RLESEGMENT_H
#define RLESEGMENT_H

class RLESegment {
public:
    RLESegment();
    RLESegment(int _x, int _length);
    int x; //position in line
    int length; //length of run
    int label; //Object label (-1 if not associated)
};

class RLERegion {
public:
    RLERegion();
    RLERegion(int l);
    void insert(int y, RLESegment& seg);
    void quick_insert(int y, RLESegment& seg);
    void quick_insert(int y, RLESegment* seg);

    void merge(RLERegion& reg);
    Rectangle<int> r;
    std::map<int, std::deque<RLESegment *> > s;
    int label; //Object label (-1 if not associated)
    bool first;
};


#endif // RLESEGMENT_H
