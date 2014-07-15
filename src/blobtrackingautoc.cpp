#include "blobtrackingautoc.h"
#include <QDebug>

#define MINW 10
#define MINH 10
/* Constructor of auto blob tracker: */
CvBlobTrackerAutoc::CvBlobTrackerAutoc(CvBlobTrackerAutoParam1* param):m_BlobList(sizeof(CvBlobTrackAuto))
{
    m_BlobList.AddFormat("i");
    m_TimesFile = NULL;
    AddParam("TimesFile",&m_TimesFile);

    m_NextBlobID = 0;
    m_pFGMask = NULL;
    m_FrameCount = 0;

    m_FGTrainFrames = param?param->FGTrainFrames:0;
    m_pFG = param?param->pFG:0;

    m_BDDel = 0;
    m_pBD = param?param->pBD:NULL;
    m_BTDel = 0;
    m_pBT = param?param->pBT:NULL;;
    m_BTReal = m_pBT?m_pBT->IsModuleName("BlobTrackerReal"):0;

    m_pBTGen = param?param->pBTGen:NULL;

    m_pBTA = param?param->pBTA:NULL;

    m_pBTPostProc = param?param->pBTPP:NULL;
    m_UsePPData = param?param->UsePPData:0;

    /* Create default submodules: */
    if(m_pBD==NULL)
    {
        m_pBD = cvCreateBlobDetectorSimple();
        m_BDDel = 1;
    }

    if(m_pBT==NULL)
    {
        m_pBT = cvCreateBlobTrackerMS();
        m_BTDel = 1;
    }

    SetModuleName("Autoc");

} /* CvBlobTrackerAuto1::CvBlobTrackerAuto1 */

/* Destructor for auto blob tracker: */
CvBlobTrackerAutoc::~CvBlobTrackerAutoc()
{
    if(m_BDDel)m_pBD->Release();
    if(m_BTDel)m_pBT->Release();
}

void CvBlobTrackerAutoc::Process(IplImage* pImg, IplImage* pMask)
{
    int         CurBlobNum = 0;
    int         i;
    IplImage*   pFG = pMask;

    /* Bump frame counter: */
    m_FrameCount++;

    /* Update BG model: */
    if(m_pFG)
    {   /* If FG detector is needed: */
        m_pFG->Process(pImg);
        pFG = m_pFG->GetMask();
    }   /* If FG detector is needed. */


    m_pFGMask = pFG; /* For external use. */

    /*if(m_pFG && m_pFG->GetParam("DebugWnd") == 1)
    {// debug foreground result
        IplImage *pFG = m_pFG->GetMask();
        if(pFG)
        {
            cvNamedWindow("FG",0);
            cvShowImage("FG", pFG);
        }
    }*/

    /* Track blobs: */
    if(m_pBT)
    {
        int i;
        m_pBT->Process(pImg, pFG);
        for(i=m_BlobList.GetBlobNum(); i>0; --i)
        {   /* Update data of tracked blob list: */
            CvBlob* pB = m_BlobList.GetBlob(i-1);
            int     BlobID = CV_BLOB_ID(pB);
            int     i = m_pBT->GetBlobIndexByID(BlobID);
            m_pBT->ProcessBlob(i, pB, pImg, pFG);
            pB->ID = BlobID;
        }
        CurBlobNum = m_pBT->GetBlobNum();
    }

    /* This part should be removed: */


    if(m_BTReal && m_pBT)
    {   /* Update blob list (detect new blob for real blob tracker): */
        int i;

        for(i=m_pBT->GetBlobNum(); i>0; --i)
        {   /* Update data of tracked blob list: */
            CvBlob* pB = m_pBT->GetBlob(i-1);
            if(pB && m_BlobList.GetBlobByID(CV_BLOB_ID(pB)) == NULL )
            {
                CvBlobTrackAuto     NewB;
                NewB.blob = pB[0];
                NewB.BadFrames = 0;
                m_BlobList.AddBlob((CvBlob*)&NewB);
            }
        }   /* Next blob. */

        /* Delete blobs: */
        for(i=m_BlobList.GetBlobNum(); i>0; --i)
        {   /* Update tracked-blob list: */
            CvBlob* pB = m_BlobList.GetBlob(i-1);
            if(pB && m_pBT->GetBlobByID(CV_BLOB_ID(pB)) == NULL )
            {
                m_BlobList.DelBlob(i-1);
            }
        }   /* Next blob. */
    }   /* Update bloblist. */

    if(m_pBTPostProc)
    {   /* Post-processing module: */
        int i;
        for(i=m_BlobList.GetBlobNum(); i>0; --i)
        {   /* Update tracked-blob list: */
            CvBlob* pB = m_BlobList.GetBlob(i-1);
            m_pBTPostProc->AddBlob(pB);
        }
        m_pBTPostProc->Process();
        for(i=m_BlobList.GetBlobNum(); i>0; --i)
        {   /* Update tracked-blob list: */
            CvBlob* pB = m_BlobList.GetBlob(i-1);
            int     BlobID = CV_BLOB_ID(pB);
            CvBlob* pBN = m_pBTPostProc->GetBlobByID(BlobID);

            if(pBN && m_UsePPData && pBN->w >= CV_BLOB_MINW && pBN->h >= CV_BLOB_MINH)
            {   /* Set new data for tracker: */
                m_pBT->SetBlobByID(BlobID, pBN );
            }

            if(pBN)
            {   /* Update blob list with results from postprocessing: */
                pB[0] = pBN[0];
            }
        }
    }   /* Post-processing module. */

    /* Blob deleter (experimental and simple): */
    if(pFG)
    {   /* Blob deleter: */
        int i;
        if(!m_BTReal)for(i=m_BlobList.GetBlobNum();i>0;--i)
        {   /* Check all blobs on list: */
            CvBlobTrackAuto* pB = (CvBlobTrackAuto*)(m_BlobList.GetBlob(i-1));
            int     Good = 0;
            int     w=pFG->width;
            int     h=pFG->height;
            CvRect  r = CV_BLOB_RECT(pB);
            CvMat   mat;
            double  aver = 0;
            double  area = CV_BLOB_WX(pB)*CV_BLOB_WY(pB);
            if(r.x < 0){r.width += r.x;r.x = 0;}
            if(r.y < 0){r.height += r.y;r.y = 0;}
            if(r.x+r.width>=w){r.width = w-r.x-1;}
            if(r.y+r.height>=h){r.height = h-r.y-1;}

            if(r.width > 4 && r.height > 4 && r.x < w && r.y < h &&
                r.x >=0 && r.y >=0 &&
                r.x+r.width < w && r.y+r.height < h && area > 0)
            {
                aver = cvSum(cvGetSubRect(pFG,&mat,r)).val[0] / area;
                /* if mask in blob area exists then its blob OK*/
                if(aver > 0.1*255)Good = 1;
            }
            else
            {
                pB->BadFrames+=2;
            }

            if(Good)
            {
                pB->BadFrames = 0;
            }
            else
            {
                pB->BadFrames++;
            }
        }   /* Next blob: */

        /* Check error count: */
        for(i=0; i<m_BlobList.GetBlobNum(); ++i)
        {
            CvBlobTrackAuto* pB = (CvBlobTrackAuto*)m_BlobList.GetBlob(i);

            if(pB->BadFrames>3)
            {   /* Delete such objects */
                /* from tracker...     */
                m_pBT->DelBlobByID(CV_BLOB_ID(pB));

                /* ... and from local list: */
                m_BlobList.DelBlob(i);
                i--;
            }
        }   /* Check error count for next blob. */
    }   /*  Blob deleter. */

    /* Update blobs: */
    if(m_pBT)
        m_pBT->Update(pImg, pFG);

    /* Detect new blob: */

    if(!m_BTReal && m_pBD && pFG && (m_FrameCount > m_FGTrainFrames) )
    {   /* Detect new blob: */
        static CvBlobSeq    NewBlobList;
        CvBlobTrackAuto     NewB;

        NewBlobList.Clear();
        if(m_pBD->DetectNewBlob(pImg, pFG, &NewBlobList, &m_BlobList))
        {   /* Add new blob to tracker and blob list: */
            int i;
            IplImage* pMask = pFG;

            /*if(0)if(NewBlobList.GetBlobNum()>0 && pFG )
            {// erode FG mask (only for FG_0 and MS1||MS2)
                pMask = cvCloneImage(pFG);
                cvErode(pFG,pMask,NULL,2);
            }*/

            for(i=0; i<NewBlobList.GetBlobNum(); ++i)
            {
                CvBlob* pBN = NewBlobList.GetBlob(i);
                pBN->ID = m_NextBlobID;

                if(pBN && pBN->w >= MINW && pBN->h >= MINH)
                {
                    CvBlob* pB = m_pBT->AddBlob(pBN, pImg, pMask );
                    if(pB)
                    {
                        NewB.blob = pB[0];
                        NewB.BadFrames = 0;
                        m_BlobList.AddBlob((CvBlob*)&NewB);
                        m_NextBlobID++;
                    }
                }
            }   /* Add next blob from list of detected blob. */

            if(pMask != pFG) cvReleaseImage(&pMask);

        }   /* Create and add new blobs and trackers. */

    }   /*  Detect new blob. */


    if(m_pBTGen)
    {   /* Run track generator: */
        for(i=m_BlobList.GetBlobNum(); i>0; --i)
        {   /* Update data of tracked blob list: */
            CvBlob* pB = m_BlobList.GetBlob(i-1);
            m_pBTGen->AddBlob(pB);
        }
        m_pBTGen->Process(pImg, pFG);
    }   /* Run track generator: */

    if(m_pBTA)
    {   /* Trajectory analysis module: */
        int i;
        for(i=m_BlobList.GetBlobNum(); i>0; i--)
            m_pBTA->AddBlob(m_BlobList.GetBlob(i-1));

        m_pBTA->Process(pImg, pFG);

    }   /* Trajectory analysis module. */


} /* CvBlobTrackerAuto1::Process */

