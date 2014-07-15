#include "paintView.h"
#include "VideoAnalysis.h"

//Get draw headers and handlers
#include "draw.h"
std::map<std::string, drawInterface *(*)(Datapool *)> paintView::drawConstructor;
QImage *paintView::defaultBg = NULL;

paintView::paintView(){
    image = NULL;
    fnumber = 0;
    setDraw();
}
paintView::paintView(Datapool *i_data, QString afterModule, int col, int row, int m_vwidth):m_data(i_data), afterModule(afterModule){
    this->column = col;
    this->row = row;
    this->indexView = row*m_vwidth + col;
    setDraw();
    image = NULL;
    fnumber = 0;
}

paintView::~paintView(){
    if(drawSequence.size() > 0) {
        std::deque<drawInterface *>::iterator it, it_end = drawSequence.end();
        for(it = drawSequence.begin(); it != it_end; it++)
            delete *it;
        drawSequence.clear();
    }
    if(image != NULL)
        delete image;
    image = NULL;
}

bool paintView::init(){
    std::deque<drawInterface *>::iterator it, it_end = drawSequence.end();
    for(it = drawSequence.begin(); it != it_end; it++)
        if(!(*it)->init())
            return false;
    return true;
}

bool paintView::setParameters(QDomNode &config){
    QDomNode m;
    QDomElement e;
    int i;
    size_t pos;
    std::deque<drawInterface *>::reverse_iterator it, it_end;

    m = config.firstChild();
    while(!m.isNull()) {
        e = m.toElement();
        std::string s = e.tagName().toStdString();
        if( (pos=s.find_first_not_of(" \t\r\n")) != std::string::npos ) {
            QDomNode p;
            s = s.substr(pos);
            i = s.size()-1;
            if(s[i] == '\n' || s[i] == '\r')
                s = s.substr(0, i);

            if(isValidDrawName(s)) {
                drawSequence.push_back(drawConstructor[s](m_data));
                it = drawSequence.rbegin();
                (*it)->setParameters(m);
            } else {
                QString ss = s.c_str();
                AppendToLog("paintView: setParameters: draw element '" + ss + "' not defined. Draw element will be ignored.");
            }
        }
        m = m.nextSibling();
    }
    return true;
}

bool paintView::draw(){
    if(m_data->currentImage != NULL) {
        int
            w = m_data->currentImage->width(),
            h = m_data->currentImage->height();
        if(image == NULL) {
            image = new QImage(w, h, QImage::Format_ARGB32);
        } else if(w != image->width() || h != image->height()) {
            delete image;
            image = new QImage(w, h, QImage::Format_ARGB32);
        }
        if(defaultBg == NULL) {
            defaultBg = new QImage(w, h, QImage::Format_ARGB32);
            initBytes(defaultBg, 0);
        }  else if(w != defaultBg->width() || h != defaultBg->height()) {
            delete defaultBg;
            defaultBg = new QImage(w, h, QImage::Format_ARGB32);
            initBytes(defaultBg, 0);
        }
    }
    if(image == NULL)
        return false;
    copyBytes(image, defaultBg);

    std::deque<drawInterface *>::iterator it, it_end = drawSequence.end();
    for(it = drawSequence.begin(); it != it_end; it++)
        //pinta cada capa asociada a la imagen apuntada por el paintView.
        if(!(*it)->draw(image))
            return false;
    return true;
}

QString paintView::getAfterModule(){
    return afterModule;
}

int paintView::getIndexView(){
    return indexView;
}

QImage *paintView::getImage(){
    return image;
}

QDomNode paintView::getParameterNode(QString pname, QDomElement& elem) {
    QDomNodeList n = elem.elementsByTagName(pname);
    return n.item(0);
}

QDomNode paintView::getParameterNode(QString pname, QDomNode& node) {
    if(node.isNull())
        return node;
    QDomNodeList n = node.toElement().elementsByTagName(pname);
    return n.item(0);
}

QString paintView::getParameterValue(QDomNode& node) {
    return node.toElement().attribute("value");
}

QString paintView::getParameterValue(QDomElement& elem) {
    return elem.attribute("value");
}

void paintView::copyBytes(QImage *im1, QImage *im2) {
    memcpy(im1->bits(),im2->bits(), im1->height()*im1->bytesPerLine());
}

void paintView::initBytes(QImage *im, uchar value) {
    memset(im->bits(), value, im->height()*im->bytesPerLine());
    if(value != 255 && (   im->format() != QImage::Format_ARGB32
                        || im->format() != QImage::Format_ARGB32_Premultiplied
                        || im->format() != QImage::Format_RGB32 ) ) {
        int i, j, h = im->height(), b = im->bytesPerLine();
        uchar *pixels = im->bits();
        for(i=0; i<h; i++)
            for(j=3; j<b; j+=4)
                pixels[i*b + j] = 255;
    }
}

bool paintView::isValidDrawName(std::string name) {
    return drawConstructor.count(name) > 0;
}

void paintView::setDisplayPaint(bool value){
    displayPaint = value;
}
bool paintView::getDisplayPaint(){
    return displayPaint;
}
