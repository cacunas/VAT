#ifndef PAINTVIEW_H
#define PAINTVIEW_H

#include "QtXml/QDomDocument"
#include "DRAW/drawInterface.h"
#include "Datapool.h"
#include <iostream>

class paintView
{
public:
    paintView();
    paintView(Datapool *i_data, QString afterModule, int col, int row, int m_vwidth);
    ~paintView();

    //Set module configuration parameters
    bool setParameters(QDomNode &config);

    //Initialization after reading parameters
    bool init();

    //Function executed at each frame
    bool draw();

    QString getAfterModule();
    int getIndexView();
    QImage *getImage();
    void setDisplayPaint(bool value);
    bool getDisplayPaint();

    void setDraw();
    static void copyBytes(QImage *im1, QImage *im2);
    static void initBytes(QImage *im1, uchar value);

    static bool isValidDrawName(std::string name);


    std::deque<drawInterface *> drawSequence;
    static std::map<std::string, drawInterface *(*)(Datapool *)> drawConstructor;
    static QImage *defaultBg;

    static QDomNode getParameterNode(QString pname, QDomElement& elem);
    static QDomNode getParameterNode(QString pname, QDomNode& node);
    static QString getParameterValue(QDomNode& node);
    static QString getParameterValue(QDomElement& elem);

    bool saveImage;
    QString saveDir;
    int fnumber;

protected:
  //Reference to external datapool:
  Datapool *m_data;

private:
    bool displayPaint;
    int column, row;
    int indexView;
    QString afterModule;
    QImage *image;
};

#endif // PAINTVIEW_H
