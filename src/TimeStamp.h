#ifndef _TIMESTAMP_H_
#define _TIMESTAMP_H_

class TimeStamp {
    public:
        TimeStamp();
        ~TimeStamp();
        int operator-(TimeStamp&); //gives millisecs diff.
        TimeStamp& operator+=(int); //adds millisecs to timestamp.
        static bool dayPassedMonth(int year, int month, int day);

        int frame_id;
        int year;
        int month;
        int day;
        int hour;
        int minute;
        int second;
        int millisecond;
};


#endif /* _TIMESTAMP_H_ */ 
