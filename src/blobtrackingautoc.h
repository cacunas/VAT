#ifndef BLOBTRACKINGAUTOC_H
#define BLOBTRACKINGAUTOC_H

#include <opencv2/opencv.hpp>
#include <opencv2/legacy/blobtrack.hpp>
#include <time.h>

/* list of Blob Detection modules */
CvBlobDetector* cvCreateBlobDetectorSimple();

/* Special extended blob structure for auto blob tracking: */
typedef struct CvBlobTrackAuto
{
    CvBlob  blob;
    int     BadFrames;
} CvBlobTrackAuto;

class CvBlobTrackerAutoc: public CvBlobTrackerAuto
{
public:
    CvBlobTrackerAutoc(CvBlobTrackerAutoParam1* param);
    ~CvBlobTrackerAutoc();
    CvBlob* GetBlob(int index){return m_BlobList.GetBlob(index);}
    CvBlob* GetBlobByID(int ID){return m_BlobList.GetBlobByID(ID);}
    int     GetBlobNum(){return m_BlobList.GetBlobNum();}
    virtual IplImage* GetFGMask(){return m_pFGMask;}
    float   GetState(int BlobID){return m_pBTA?m_pBTA->GetState(BlobID):0;}
    const char*   GetStateDesc(int BlobID){return m_pBTA?m_pBTA->GetStateDesc(BlobID):NULL;}
    /* Return 0 if trajectory is normal;
       return >0 if trajectory abnormal. */
    void Process(IplImage* pImg, IplImage* pMask = NULL);
    void Release(){delete this;}

private:
    IplImage*               m_pFGMask;
    int                     m_FGTrainFrames;
    CvFGDetector*           m_pFG; /* Pointer to foreground mask detector module. */
    CvBlobTracker*          m_pBT; /* Pointer to Blob tracker module. */
    int                     m_BTDel;
    int                     m_BTReal;
    CvBlobDetector*         m_pBD; /* Pointer to Blob detector module. */
    int                     m_BDDel;
    CvBlobTrackGen*         m_pBTGen;
    CvBlobTrackPostProc*    m_pBTPostProc;
    int                     m_UsePPData;
    CvBlobTrackAnalysis*    m_pBTA; /* Blob trajectory analyser. */
    CvBlobSeq               m_BlobList;
    int                     m_FrameCount;
    int                     m_NextBlobID;
    const char*             m_TimesFile;
};

#endif // BLOBTRACKINGAUTOC_H
