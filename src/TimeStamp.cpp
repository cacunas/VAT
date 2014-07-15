#include "TimeStamp.h"
#include <cstring>

TimeStamp::TimeStamp() {
    memset(this, 0, sizeof(TimeStamp));
    this->millisecond = -1;
}

TimeStamp::~TimeStamp() {}

int TimeStamp::operator-(TimeStamp& ts) { //gives millisecs diff., suppossing lvalue is higher
    int cdiff, tdiff = 0;
    tdiff = 1000*(this->second - ts.second) + this->millisecond - ts.millisecond;
    if(this->minute > ts.minute) {
        cdiff = this->minute - ts.minute;
        tdiff += 60000*cdiff;
    } else if(this->minute < ts.minute) {
        cdiff = 60 + this->minute - ts.minute;
        tdiff += 60000*cdiff;
    }
    return tdiff;
}

TimeStamp& TimeStamp::operator+=(int ms) { //adds millisecs to timestamp.
    this->millisecond += ms;
    if(this->millisecond >= 1000) {
        this->millisecond -= 1000;
        this->second++;
        if(this->second >= 60) {
            this->second -= 60;
            this->minute++;
            if(this->minute >= 60) {
                this->minute -= 60;
                this->hour++;
                if(this->hour >= 24) {
                    this->hour -= 24;
                    this->day++;
                    if(dayPassedMonth(this->year, this->month, this->day)) {
                        this->day = 1;
                        this->month++;
                        if(this->month > 12) {
                            this->month = 1;
                            this->year++;
                        }
                    }
                }
            }
        }
    }

    return *this;
}

bool TimeStamp::dayPassedMonth(int year, int month, int day) {
    switch(month) {
    case 1: case 3: case 5: case 7: case 8: case 10: case 12:
        return day <= 31 ? false : true;
        break;
    case 4: case 6: case 9:case 11:
        return day <= 30 ? false : true;
        break;
    case 2:
        if(year % 4 == 0)
            return day <= 29 ? false : true;
        else
            return day <= 28 ? false : true;
        break;
    default:
        return false;
    }
}

