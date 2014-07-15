#ifndef RELIABLEMOBILELIST_OBJECT2D_H
#define RELIABLEMOBILELIST_OBJECT2D_H

#include "blob.h"
#include "ReliableMobileObject2D.h"
#include <iostream>
#include <map>
#include <deque>


class ReliableMobileObjectList2D {
   
    private:
        std::deque<SpReliableMobileObject2D> reliableMobileObjects;
    
    public:
        ReliableMobileObjectList2D();
        ~ReliableMobileObjectList2D();
        std::deque<SpReliableMobileObject2D>::iterator begin();
        std::deque<SpReliableMobileObject2D>::iterator end();
        SpReliableMobileObject2D operator[](int);
        unsigned int size();
        bool empty();
        void clear();
        void insert(SpReliableMobileObject2D);
        void insert(std::deque<SpReliableMobileObject2D>::iterator pos, std::deque<SpReliableMobileObject2D>::iterator f, std::deque<SpReliableMobileObject2D>::iterator l);
        std::deque<SpReliableMobileObject2D>::iterator erase(std::deque<SpReliableMobileObject2D>::iterator pos);
        friend std::ostream& operator<<(std::ostream&, ReliableMobileObjectList2D&);
};


typedef ReliableMobileObjectList2D* rmobile_object_list2D_t;
typedef QSharedPointer<ReliableMobileObjectList2D> SpReliableMobileObjectList2D;

#endif	
