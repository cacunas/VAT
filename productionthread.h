#ifndef PRODUCTIONTHREAD_H
#define PRODUCTIONTHREAD_H

#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include "VideoAnalysis.h"

class ProductionThread : public QThread {
    Q_OBJECT

    public:
        ProductionThread(VideoAnalysis *i_va, QObject *parent = 0);
        ~ProductionThread();
        void run();
        bool init();
        VideoAnalysis *m_videoAnalysis;
        bool newVideoAnalysisInstance;
        void do_play();
        void do_next();
        void do_pause();
        void do_init();
        void do_end();
        void load_done();

        //Execution flags
        bool done;
        bool paused;
        bool waiting;
        bool next;
    signals:
        void load_data();
        void bad_init();

};

#endif // PRODUCTIONTHREAD_H
