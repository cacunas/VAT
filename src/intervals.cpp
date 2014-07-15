#include "intervals.h"
#include <cstring>
#include <cmath>

Interval::Interval() {
    length = x1 = x2 = 0.0;
    is_null = false;
}

Interval::Interval(double i_x1, double i_x2) {
    if(x1 <= x2) {
        x1 = i_x1;
        x2 = i_x2;
    } else {  //If values are inverted in order
        x1 = i_x2;
        x2 = i_x1;
    }
    length = fabs(x2 - x1);
    is_null = false;
}

Interval::Interval(Interval *i) {
    memcpy(this, i, sizeof(Interval));
}


Interval::~Interval() {}

void Interval::newInterval(interval_t i, double i_x1, double i_x2) {
    if(i_x1 <= i_x2) {
        i->x1 = i_x1;
        i->x2 = i_x2;
    } else {  //If values are inverted in order
        i->x1 = i_x2;
        i->x2 = i_x1;
    }
    i->length = fabs(i->x2 - i->x1);
    i->is_null = false;
}


void Interval::copyInterval(interval_t src) {
    if (src == NULL)
        return;
    memcpy(this, src, sizeof(Interval));
}

void Interval::copyInterval(interval_t dest, interval_t src) {
    if (src == NULL || dest == NULL)
      return;
    memcpy(dest, src, sizeof(Interval));
}
  
void Interval::null() {
    length = x1 = x2 = 0.0;
    is_null = true;
}

//Returns intersection interval if it exist, NULL if there is no.
bool Interval::intersect(interval_t result, interval_t interval1, interval_t interval2) {

    double  x1 = INTERVAL_X1(interval1),
            x2 = INTERVAL_X2(interval1),
            y1 = INTERVAL_X1(interval2),
            y2 = INTERVAL_X2(interval2);
    if(x1 > y2) {
        result->null();
        return false;
    }
    if(x1 < y1) {
        if(x2 < y1) {
            result->null();
            return false;
        }
        if(x2 > y2)
            newInterval(result, y1, y2);
        else
            newInterval(result, y1, x2);
    } else if(x2 < y2) // [x1,x2] in [y1,y2]
        newInterval(result, x1, x2);
    else
        newInterval(result, x1, y2);
    
    return true;
}

//Returns union interval if intersection interval is not NULL, else it returns NULL.
void Interval::unifyIfIntersect(interval_t result, interval_t interval1, interval_t interval2) {
    double x1 = INTERVAL_X1(interval1),
           x2 = INTERVAL_X2(interval1),
           y1 = INTERVAL_X1(interval2),
           y2 = INTERVAL_X2(interval2);

    intersect(result, interval1, interval2);

    if(INTERVAL_IS_NULL(result))
        return;

    if(x1 < y1) {
        if(x2 > y2)
            newInterval(result, x1, x2);
        else
            newInterval(result, x1, y2);
    } else if(x2 < y2) //if x1 in [y1,y2]
        newInterval(result, y1, y2);
    else
        //if x1 in [y1,y2] and x2 out [y1,y2]
        return newInterval(result, y1, x2);
}

