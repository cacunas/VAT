#ifndef RMMHYPHOTHESISSETLIST_H
#define RMMHYPHOTHESISSETLIST_H

#include "src/blob.h"
#include "RMMHypothesisSet.h"
#include <iostream>
#include <map>
#include <deque>


class RMMHypothesisSetList {
   
    private:
        std::deque<SpRMMHypothesisSet> RMMHypothesisSets;
    
    public:
        RMMHypothesisSetList();
        ~RMMHypothesisSetList();
        std::deque<SpRMMHypothesisSet>::iterator begin();
        std::deque<SpRMMHypothesisSet>::iterator end();
        SpRMMHypothesisSet operator[](int);
        unsigned int size();
        bool empty();
        void clear();
        void insert(SpRMMHypothesisSet);
        void insert(std::deque<SpRMMHypothesisSet>::iterator pos, std::deque<SpRMMHypothesisSet>::iterator f, std::deque<SpRMMHypothesisSet>::iterator l);
        std::deque<SpRMMHypothesisSet>::iterator erase(std::deque<SpRMMHypothesisSet>::iterator pos);
        friend std::ostream& operator<<(std::ostream&, RMMHypothesisSetList&);
};


typedef RMMHypothesisSetList* hypothesis_set_list_t;
typedef QSharedPointer<RMMHypothesisSetList> SpRMMHypothesisSetList;

#endif	
