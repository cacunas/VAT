#include "productionthread.h"
#include <QTime>

//QMutex outmutex;
QMutex pausemutex;
QMutex datapoolmutex;
QMutex donemutex;
QMutex waitmutex;
QMutex lockpausemutex;
QMutex lockwaitmutex;
QWaitCondition pauseCondition;
QWaitCondition waitCondition;

ProductionThread::ProductionThread(VideoAnalysis *i_va, QObject *parent) {
    m_videoAnalysis = i_va;
    newVideoAnalysisInstance = true;
    done = false;
}

ProductionThread::~ProductionThread() {
}

bool ProductionThread::init(){
    if( newVideoAnalysisInstance ){
        newVideoAnalysisInstance = false;
        if(!m_videoAnalysis->setParameters()) {
            newVideoAnalysisInstance = true;
            return false;
        }
        if(!m_videoAnalysis->init()) {
            newVideoAnalysisInstance = true;
            return false;
        }
    }
    return true;
}

void ProductionThread::run(){
    if(!init()) {
        emit bad_init();
        return;
    }

    bool ex;

    pausemutex.lock(); //Initially paused
    paused = true;
    pausemutex.unlock();
    waitmutex.lock();
    waiting = false; // Not waiting for display update
    waitmutex.unlock();


    forever {

        pausemutex.lock(); //If paused or next button it must stop
        if(paused) {
            pausemutex.unlock();
            lockpausemutex.lock();
            pauseCondition.wait(&lockpausemutex);
            lockpausemutex.unlock();
        } else
            pausemutex.unlock();

        waitmutex.lock(); //Control if it is waiting for printing results
        if(waiting) {
            waitmutex.unlock();

            lockwaitmutex.lock();
            waitCondition.wait(&lockwaitmutex);
            lockwaitmutex.unlock();
        } else
            waitmutex.unlock();

        donemutex.lock();
        if(done) {
            donemutex.unlock();
            break;
        }
        donemutex.unlock();

        datapoolmutex.lock(); //Modifying datapool
        ex = m_videoAnalysis->execute();
        datapoolmutex.unlock();

        if(ex){ //If successful, execution will have to wait for data ready
            waitmutex.lock();
            waiting = true;
            waitmutex.unlock();
            emit load_data();
        } else {
            pausemutex.lock();
            paused = true;
            pausemutex.unlock();
            emit bad_init();
        }

    }
}

    //outmutex.lock();
    //std::cout << "production: run: End of run. ID:" << QThread::currentThreadId() << std::endl;
    //outmutex.unlock();

void ProductionThread::do_play() {
    pausemutex.lock();
    paused = false;
    pausemutex.unlock();
    pauseCondition.wakeAll();
}

void ProductionThread::do_end() {
    donemutex.lock();
    done = true;
    donemutex.unlock();
    pauseCondition.wakeAll();
}

void ProductionThread::do_next() {

    if(!init()) {
        emit bad_init();
        return;
    }

    datapoolmutex.lock(); //Modifying datapool
    int elapsed;
    QTime t;
    t.start();
    bool ex = m_videoAnalysis->execute();
    elapsed = t.elapsed();
    //std::cout << "Next elapsed time: " << elapsed << std::cout;

    datapoolmutex.unlock();
    if(ex)
        emit load_data();
    else
        emit bad_init();

}

void ProductionThread::do_pause() {
    pausemutex.lock();
    paused = true;
    pausemutex.unlock();
}

void ProductionThread::load_done() {
    waitmutex.lock();
    waiting = false;
    waitmutex.unlock();
    waitCondition.wakeAll();
}



void ProductionThread::do_init(){
    newVideoAnalysisInstance = true;
    m_videoAnalysis->resetModules();
    do_next();
}

