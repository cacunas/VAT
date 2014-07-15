#ifndef __INTERVALS_H__
#define __INTERVALS_H__

class Interval;

typedef Interval *interval_t;

class Interval {
    public:
        double x1;
        double x2;
        double length;
        bool is_null;

        Interval();
        Interval(double, double);
        Interval(Interval *);
        ~Interval();

        //Initialize an interval with two values
        static void newInterval(interval_t, double, double);
        //Copy information from parameter
        void copyInterval(interval_t);
        //Copy information from second to first parameter.
        static void copyInterval(interval_t, interval_t);
        //Store intersection interval of i1 and i2 if it exist, null interval if there is no. Returns 1 if there is intersection and 0 if not
        static bool intersect(interval_t result, interval_t i1, interval_t i2);
        //Returns union interval result if intersection interval is not NULL, else it returns NULL.
        static void unifyIfIntersect(interval_t result, interval_t interval1, interval_t interval2);
        //Sets data representing null interval
        void null();
};
  
  
#define INTERVAL_LENGTH(_interval)	((_interval)->length)
#define INTERVAL_IS_NULL(_interval)	((_interval)->is_null)
#define INTERVAL_X1(_interval)	((_interval)->x1)
#define INTERVAL_X2(_interval)	((_interval)->x2)
  
#endif
