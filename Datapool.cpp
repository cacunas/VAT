#include "Datapool.h"
#include <QDate>
//Init Data:
Datapool::Datapool() {
    init();
}

//INIT DATAPOOL ELEMENTS HERE!
void Datapool::init(){
    frameNumber = 0;
    currentImage = NULL;
    grayImage = NULL;
    currentHeader = NULL;
    maskImage = NULL;
    bgImage = NULL;
    fgImage = NULL;
    labelMatrix = NULL;
    currentDiffImage = NULL;
    previousDiffImage = NULL;
    rFgImage = NULL;
    persoImage = NULL;
    previousImage = NULL;
    personalityPCAImage = new QImage(1024, 768, QImage::Format_RGB888);
    memset(personalityPCAImage->bits(), 255, personalityPCAImage->bytesPerLine()*personalityPCAImage->height());
    eventLearning = miles::SpIncrementalEventLearning();
    availableAttributes = miles::SpLearningAttributes();
    sceneModel = NULL;
    grayScaleTable = setGrayScaleTable();
    degu = NULL;
    runModules = true;
}

//PUT CLEANING DATAPOOL ELEMENTS HERE!
void Datapool::clear() {
    if(currentImage != NULL)
        delete currentImage;
    if(grayImage != NULL)
        delete grayImage;
    if(currentHeader != NULL)
        delete currentHeader;
    if(bgImage != NULL)
        delete bgImage;
    if(fgImage != NULL)
        delete fgImage;

    if(rFgImage != NULL)
        delete rFgImage;
    if(currentDiffImage != NULL)
        delete currentDiffImage;
    if(previousDiffImage != NULL)
        delete previousDiffImage;
    if(previousImage != NULL)
        delete previousImage;
    if(sceneModel != NULL)
        delete sceneModel;
    if(personalityPCAImage != NULL)
        delete personalityPCAImage;
    currentImage = NULL;
    currentHeader = NULL;
    bgImage = NULL;
    fgImage = NULL;
    sceneModel = NULL;

    objectModels.clear();
    blobs.clear();
    objects.clear();
    objects2D.clear();
    RMMobjects.clear();
    RLEFGImage.clear();
    RLERegions.clear();
    //TEMP
    modelsToFollow.clear();

    if(degu != NULL)
        delete degu;
    degu = NULL;

    if(persoImage != NULL)
        delete persoImage;
    persoImage = NULL;

    if(grayScaleTable != NULL)
        delete grayScaleTable;
    grayScaleTable = NULL;

	runModules = true;
}



void Datapool::appendToLog(const QString& toLog) {
        emit logString(toLog);
}

void Datapool::pauseApp() {
        emit pause();
}

Datapool::~Datapool() {
    delete grayScaleTable;
    clear();
}

QString Datapool::getCurrentDateTimeString() {
    QString s = "";
    QDate d;
    QTime t;
    s = d.currentDate().toString("dd.MM.yyyy") + "-" + t.currentTime().toString("hh.mm.ss");
    return s;
}

QVector<QRgb> *Datapool::setGrayScaleTable() {
    int i;
    QVector<QRgb> *gsTable = new QVector<QRgb>(256);
    for(i=0;i<256;i++)
        (*gsTable)[i] = qRgb(i,i,i);
    return gsTable;
}
