#ifndef XMLCOMMON_H
#define XMLCOMMON_H

#include <QDomNode>

class XmlCommon
{
public:
    XmlCommon();
    static QDomNode getParameterNode(QString pname, QDomElement& elem);
    static QDomNode getParameterNode(QString pname, QDomNode& node);
    static QString getParameterValue(QDomNode& node);
    static QString getParameterValue(QDomElement& elem);
    static QString getParameterValue(QDomNode& node, QString name);
    static QString getParameterValue(QDomElement& elem, QString name);

};

#endif // XMLCOMMON_H
