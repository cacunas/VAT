#ifndef TEXTTHREAD_H
#define TEXTTHREAD_H
#include <QTextEdit>
#include "productionthread.h"

class playThread : public QThread {
    Q_OBJECT

    public:
        //textThread(QTextEdit *aQObject *parent=0);
        playThread(VideoAnalysis *i_va, char *config_file, QObject *parent = 0);
        void run();
        void unlock_pauseCondition();

        VideoAnalysis *m_videoAnalysis;
        ProductionThread *producer;

    signals:
        void load_data();
        void bad_init();

    private slots:
        void start_production();
        void continue_production();
        void pause_production();
        void finish_production();
        void send_load_data();
        void send_bad_init();
        void send_init_next();
        void send_make_next();
};
#endif
