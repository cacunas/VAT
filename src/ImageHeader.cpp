#include "ImageHeader.h"
#include "TimeStamp.h" 
#include "VideoAnalysis.h"
#include <iostream>
#include <cstring>
#include <QFile>

int ImageHeader::days_per_month[12] = {31,28,31,30,31,30,31,31,30,31,30,31};

ImageHeader::ImageHeader() {
    setNullHeader();
}

ImageHeader::~ImageHeader() { }

void ImageHeader::setNullHeader() {
    memset(&ts, 0, sizeof(TimeStamp));
    ts.millisecond = -1;
    jpeg_header_string = "";
    ts_present = false;
    ts_diff = 80; //default
}

// Opens filename file and reads its header (Application marker segment,
// it starts after a FF E0).
// This header is given back as string. If something goes wrong, empty
// string is given back.
void ImageHeader::readJpegHeader(QString& filename) {

    QFile file(filename);
    if( !file.open(QIODevice::ReadOnly) ) {
        AppendToLog(   "Error opening file. The image file '"
                                       + filename
                                       + "' could not be opened for header extraction." );
        setNullHeader();
        return;
    }

    char buffer[2] = {0};    
    int  buf_size;

    do {
        buffer[0] = buffer[1];           // Shift
        if(file.read(&buffer[1], 1) <= 0) {
            AppendToLog(   "ImageHeader: The image file '"
                                           + filename
                                           + "' does not contain a header with timestamp." );
            setNullHeader();
            return;
        }
    }  while (buffer[0] != '\xff' || buffer[1] != '\xe0');   // Marker bytes found

    file.read(&buffer[0], 2);            // Two bytes: header size

    // These lines are the conversion from hex (char) to int
    QString hex;
    int from_hex = (hex.sprintf("%d", buffer[0])).toInt();
    buf_size = 256 * from_hex;
    from_hex = (hex.sprintf("%d", buffer[1])).toInt();
    buf_size += from_hex;

    if(buf_size <= 0) {
        AppendToLog(   "ImageHeader: The image file '"
                                       + filename
                                       + "' does not contain a header with timestamp or an empty header." );
        setNullHeader();
        return;
    }
    char *header = new char[buf_size + 1];
    file.read(header, buf_size);
    header[buf_size] = '\0';

    jpeg_header_string = header;

    file.close();
    delete[] header;

}

// Writes the header with the data in the parameters.
// First input parameter is the milliseconds increment per frame.
// Second one is the camera ID (on 3 bytes + '\0')
// Time, date and framecount are automatically generated.
// y,m,d,hh,mm,ss,mss,frm are the starting values for time, date and frame count.
// They are used only the first call (the others uses ms_increment_per_frame to
// compute the new time/date
void ImageHeader::jpegHeaderStringAutomaticGeneration(
                                            QString& header,
                                            double ms_increment_per_frame,
                                            QString& camera_id,
                                            int y, int m, int d,
                                            int hh, int mm, int ss, double mss, int frm) {
    if (header == "") {
        std::cout << "jpegHeaderStringAutomaticGeneration: error: An empty buffer passed to jpeg_header_string_automatic_generation function.Exiting..." << std::endl;
        return;
    }

    if (ts.year == -1) {
        // Variable initialization
        ts.year = y;
        ts.month = m;
        ts.day = d;
        ts.hour = hh;
        ts.minute = mm;
        ts.second = ss;
        ts.millisecond = mss;
        ts.frame_id = frm;
    } else {
        ts.millisecond += ms_increment_per_frame;
        ts.frame_id++;
        if(ts.millisecond >= 1000) {
            ts.millisecond -= 1000;
            ts.second++;
            if(ts.second >= 60) {
                ts.second -= 60;
                ts.minute++;
                if(ts.minute >= 60) {
                    ts.minute -= 60;
                    ts.hour++;
                    if(ts.hour >= 24) {
                        ts.hour -= 24;
                        ts.day++;
                        if (ts.year % 400 == 0 || (ts.year % 4 == 0 && ts.year % 100 != 0))
                            // February has 29 days...
                            days_per_month[1] = 29;
                        else
                            days_per_month[1] = 28;

                        // month-1 because array starts at 0!
                        if (ts.day > days_per_month[ts.month-1]) {
                            ts.day -= days_per_month[ts.month-1];
                            ts.month++;
                            if(ts.month > 12) {
                                ts.month -= 12;
                                ts.year++;
                            }
                        }
                    }
                }
            }
        }
    }

    // Buffer string modification with given or statically stored date, time and frame number
    header.sprintf("%04d%02d%02d%02d%02d%02d%03d%06d%s", ts.year, ts.month, ts.day, ts.hour, ts.minute, ts.second, ts.millisecond, ts.frame_id, camera_id.toStdString().c_str());
}

// Opens 'filename' file and write its header (Application marker segment,
// it starts after a FF E0) using jpeg_header_string content. The existing application
// marker segment is replaced.
void replaceJpegHeader(const QString& filename);

void replaceOrInsertJpegHeader(const QString& filename);

// Takes the jpeg header (given by function readJpegHeader) and converts
// it into a TimeStamp instance. Camera Id is given back in CamId string.
// If something goes wrong, ts_present is set to false.
void ImageHeader::jpegHeaderConvert(QString header, QString& camId) {
    QString buffer;
    int pos = 0;
    // Year
    buffer = header.mid(pos, 4);
    pos += 4;
    ts.year = buffer.toInt();

    // Month
    buffer = header.mid(pos, 2);
    pos += 2;
    ts.month = buffer.toInt();
    // Day
    buffer = header.mid(pos, 2);
    pos += 2;
    ts.day = buffer.toInt();
    // Hour
    buffer = header.mid(pos, 2);
    pos += 2;
    ts.hour = buffer.toInt();
    // Minute
    buffer = header.mid(pos, 2);
    pos += 2;
    ts.minute = buffer.toInt();
    // Second
    buffer = header.mid(pos, 2);
    pos += 2;
    ts.second = buffer.toInt();
    // Millisecond
    buffer = header.mid(pos, 3);
    pos += 3;
    ts.millisecond = buffer.toInt();
    // Framecount
    buffer = header.mid(pos, 6);
    pos += 6;
    ts.frame_id = buffer.toInt();
    // Camera ID
    camId = header.mid(pos, 3);
}


