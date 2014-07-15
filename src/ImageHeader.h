// This file contains the prototypes of functions reading
// a jpeg header (Application marker segment) and
// converting it in TimeStamp instances.
 
#ifndef __IMAGE_HEADER_H__
#define __IMAGE_HEADER_H__

#include <QString>
#include "TimeStamp.h"

class ImageHeader {
    public:
        ImageHeader();
        ~ImageHeader();

        static int days_per_month[12];

        QString jpeg_header_string; //Stores header as string. Empty string
                                        //if not present.
        QString m_camId;
        TimeStamp ts;       // Stores current image time stamp
        bool ts_present;    // Indicates if the timestamp is present in images
        int ts_diff;        // Default milliseconds difference between frames
                            // set if no timestamp is found. The value comes from a
                            // manually defined parameter (see AcquisitionModule)
        // Opens filename file and reads its header (Application marker segment,
        // it starts after a FF E0).
        // This header is given back as string. If something goes wrong, empty
        // string is given back.
        void readJpegHeader(QString& filename);

        // Takes the jpeg header (given by function readJpegHeader) and converts
        // it into a TimeStamp instance. Camera Id is given back in CamId string.
        // If something goes wrong, ts_present is set to false.
        void jpegHeaderConvert(QString header, QString& camId);


        //Set values for representing a null header
        void setNullHeader();

        // Writes the jpeg_header_string with the data in the parameters.
        // First input parameter is the milliseconds increment per frame.
        // Second one is the camera ID (on 3 bytes + '\0')
        // Time, date and framecount are automatically generated.
        // y,m,d,hh,mm,ss,mss,frm are the starting values for time, date and frame count.
        // They are used only the first call (the others uses ms_increment_per_frame to
        // compute the new time/date
        void jpegHeaderStringAutomaticGeneration(
                                            QString& header,
                                            double ms_increment_per_frame,
                                            QString& camera_id,
                                            int y, int m, int d,
                                            int hh, int mm, int ss, double mss, int frm);

};

#endif // __IMAGE_HEADER_H__
