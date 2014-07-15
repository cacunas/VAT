#include "BlobFilteringModule.h"
#include "image_display.h"

#include <errno.h>
#include <iostream>
#include <vector>
#include <src/blob.h>

BlobFilteringModule::BlobFilteringModule(Datapool *i_data): ModuleInterface(i_data), borderConstraintsSet(false) {}

BlobFilteringModule::~BlobFilteringModule() {}

bool BlobFilteringModule::setParameters(QDomNode& config){
    QDomNode n;
    //int intT;

    if(config.isNull()) { //Parameter set for module not defined
        m_minArea = 4;
        m_minHeight = 2;
        m_minWidth = 2;
        m_ignoreBorderRate = 0.0;
        m_delInForeground = true;
        AppendToLog("BlobFilteringModule: Warning: Configuration '" + m_data->m_config_file + "' file does not define an entry BlobFilteringModule. Taking defaults...");
    } else {
        if( ( n = XmlCommon::getParameterNode("MinArea", config) ).isNull() ) {
            AppendToLog("BlobFilteringModule: Warning: Configuration '" + m_data->m_config_file + "' file does not define an entry 'MinArea' for BlobFilteringModule. Taking default value (4).");
            m_minArea = 4;
        } else
            m_minArea = XmlCommon::getParameterValue(n).toInt();

        if( ( n = XmlCommon::getParameterNode("MinHeight", config) ).isNull() ) {
            AppendToLog("BlobFilteringModule: Warning: Configuration '" + m_data->m_config_file + "' file does not define an entry 'MinHeight' for BlobFilteringModule. Taking default value (2).");
            m_minHeight = 2;
        } else
            m_minHeight = XmlCommon::getParameterValue(n).toInt();

        if( ( n = XmlCommon::getParameterNode("MinWidth", config) ).isNull() ) {
            AppendToLog("BlobFilteringModule: Warning: Configuration '" + m_data->m_config_file + "' file does not define an entry 'MinWidth' for BlobFilteringModule. Taking default value (2).");
            m_minWidth = 2;
        } else
            m_minWidth = XmlCommon::getParameterValue(n).toInt();

        if( ( n = XmlCommon::getParameterNode("IgnoreBorderRate", config) ).isNull() ) {
            AppendToLog("BlobFilteringModule: Warning: Configuration '" + m_data->m_config_file + "' file does not define an entry 'IgnoreBorderRate' for BlobFilteringModule. Taking default value (0.0).");
            m_ignoreBorderRate = 0.0;
        } else
            m_ignoreBorderRate = XmlCommon::getParameterValue(n).toDouble();

        if( ( n = XmlCommon::getParameterNode("DeleteInForeground", config) ).isNull() ) {
            AppendToLog("BlobFilteringModule: Warning: Configuration '" + m_data->m_config_file + "' file does not define an entry 'DeleteInForeground' for BlobFilteringModule. Taking default value (true).");
            m_delInForeground = true;
        } else
            m_delInForeground = (XmlCommon::getParameterValue(n) == "true") ? true : false;
    }

    //Setea la lista de parametros y la lista de tipos de parametros utilizadas en el ParameterDialog.
    addParameter("MinArea", QString::number(m_minArea), "int");
    addParameter("MinHeight", QString::number(m_minHeight), "int");
    addParameter("MinWidth", QString::number(m_minWidth), "int");
    addParameter("IgnoreBorderRate", QString::number(m_ignoreBorderRate), "int");
    //addParameter("DeleteInForeground", QString::number(m_ignoreBorderRate), "bool");

    return true;
}

bool BlobFilteringModule::updateParameters(){
    parameter *minA, *minH, *minW, *ibr;
    minA = getParameter("MinArea");
    minH = getParameter("MinHeight");
    minW = getParameter("MinWidth");
    ibr = getParameter("IgnoreBorderRate");

    if( minA == 0 || minH == 0 || minW == 0 || ibr == 0 ||
            minA->value.toInt() < 0 || minH->value.toInt() < 0 || minW->value.toInt() < 0)
        return false;
    m_minArea = minA->value.toInt();
    m_minHeight = minH->value.toInt();
    m_minWidth = minW->value.toInt();
    m_ignoreBorderRate = ibr->value.toDouble();
    return true;
}

bool BlobFilteringModule::init() {
    return true;
}

bool BlobFilteringModule::run(){

    if(!borderConstraintsSet) {
        if( m_data->currentImage != NULL) {
            m_imgWidth = m_data->currentImage->width();
            m_imgHeight = m_data->currentImage->height();
            m_numPixWidth = m_imgWidth * m_ignoreBorderRate;
            m_numPixHeight = m_imgHeight * m_ignoreBorderRate;
            borderConstraintsSet = true;
        }
    }

    AppendToLog("BlobFilteringModule: Blobs number before: " +  QString::number(m_data->blobs.size()));

    if(!m_data->blobs.empty()) {
        Blob *blob;
        int i = 0;
        std::vector<Blob>::iterator iter = m_data->blobs.begin();
        while(iter != m_data->blobs.end()) {
            blob = &*iter;
            if( blobOK(blob) ) {
                blob->id = i++;
                iter++;
            } else {
                if(m_delInForeground)
                    deleteBlobFromForeground(blob);
                iter = m_data->blobs.erase(iter);
            }
        }
    }
    AppendToLog("BlobFilteringModule: Blobs number after:" +  QString::number(m_data->blobs.size()));
    return true;
}

void BlobFilteringModule::deleteBlobFromForeground(Blob *blob) {

    int label = blob->component_label;
    int *labels = m_data->labelMatrix;

    if(label < 0 || labels == NULL)
        return;

    Rectangle<int> &rect = blob->bbox;
    QImage *f = m_data->fgImage;
    uchar *pixels = f->bits();
    int x, y, x0 = rect.xleft, y0 = rect.ytop, w = f->width(),
        x1 = std::min(w - 1, x0 + rect.width),
        y1 = std::min(f->height() - 1, y0 + rect.height),
        bpl = f->bytesPerLine();

    for(y = y0; y <= y1; y++)
        for(x = x0; x <= x1; x++)
            if(pixels[y*bpl + x] != 0 && labels[y*w + x] == label)
                pixels[y*bpl + x] = 0;
}


bool BlobFilteringModule::blobOK(Blob *blob) {
    Rectangle<int> *rect = &blob->bbox;
    int w = rect->width, h = rect->height;

    //Dimensional conditions
    if(w < m_minWidth)
        return false;
    if(h < m_minHeight)
        return false;
    if(w*h < m_minArea)
        return false;

    //Border conditions
    if(rect->xleft >= m_imgWidth - m_numPixWidth - 1)
        return false;
    if(rect->xright <= m_numPixWidth)
        return false;
    if(rect->ytop >= m_imgHeight - m_numPixHeight - 1)
        return false;
    if(rect->ybottom <= m_numPixHeight)
        return false;

    return true;
}
