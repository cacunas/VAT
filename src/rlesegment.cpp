#include "rlesegment.h"

RLESegment::RLESegment():x(0),length(0),label(-1) {
}

RLESegment::RLESegment(int _x, int _length):x(_x), length(_length) {
}

RLERegion::RLERegion() {
    first = true;
}

RLERegion::RLERegion(int l):label(l) {
    memset(&r, 0, sizeof(Rectangle<int>));
    first = true;
}

void RLERegion::insert(int y, RLESegment &seg){
    seg.label = label;
    s[y].push_back(&seg);

    int right = seg.x + seg.length - 1;
    if(first) {
        first = false;
        r.ytop = y;
        r.xleft = seg.x;
        r.xright = right;
    } else {
        if( r.xleft > seg.x)
            r.xleft = seg.x;
        if( r.xright < right)
            r.xright = right;
    }
    r.ybottom = y;
}

void RLERegion::quick_insert(int y, RLESegment &seg){
    seg.label = label;
    s[y].push_back(&seg);
}

void RLERegion::quick_insert(int y, RLESegment *seg){
    seg->label = label;
    s[y].push_back(seg);
}


void RLERegion::merge(RLERegion &reg) {
    r.absorbRectangle(reg.r);

    std::map<int, std::deque<RLESegment *> >::iterator r_it, r_end = reg.s.end();
    std::deque<RLESegment *>::iterator seg_it, seg_end;

    for(r_it = reg.s.begin(); r_it != r_end; r_it++) {
        seg_end = r_it->second.end();
        for(seg_it = r_it->second.begin(); seg_it != seg_end; seg_it++)
            quick_insert(r_it->first, *seg_it);
    }
}
