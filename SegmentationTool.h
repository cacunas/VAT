#ifndef SEGMENTATIONTOOL_H
#define SEGMENTATIONTOOL_H

#include <map>
#include <string>
#include <QImage>
#include <QDir>

class SegmentationTool{
public:


    static void chromaticSegmentation(QImage * currentImage,QImage *result, QImage *result_c, QImage *maskImage,bool m_mask,int minGrade,int maxGrade, int minSaturation, int maxLight);


};


#endif // SEGMENTATIONTOOL_H
