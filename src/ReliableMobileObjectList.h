#ifndef RELIABLEMOBILELIST_OBJECT_H
#define RELIABLEMOBILELIST_OBJECT_H

#include "blob.h"
#include "ReliableMobileObject.h"
#include <iostream>
#include <map>
#include <deque>


class ReliableMobileObjectList {
   
    private:
        std::deque<SpReliableMobileObject> reliableMobileObjects;
    
    public:
        ReliableMobileObjectList();
        ~ReliableMobileObjectList();
        std::deque<SpReliableMobileObject>::iterator begin();
        std::deque<SpReliableMobileObject>::iterator end();
        SpReliableMobileObject operator[](int);
        unsigned int size();
        bool empty();
        void clear();
        void insert(SpReliableMobileObject);
        void insert(std::deque<SpReliableMobileObject>::iterator pos, std::deque<SpReliableMobileObject>::iterator f, std::deque<SpReliableMobileObject>::iterator l);
        std::deque<SpReliableMobileObject>::iterator erase(std::deque<SpReliableMobileObject>::iterator pos);
        friend std::ostream& operator<<(std::ostream&, ReliableMobileObjectList&);
};


typedef ReliableMobileObjectList* rmobile_object_list_t;
typedef QSharedPointer<ReliableMobileObjectList> SpReliableMobileObjectList;

#endif	
