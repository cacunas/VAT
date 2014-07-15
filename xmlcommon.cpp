#include "xmlcommon.h"

XmlCommon::XmlCommon(){ }

QDomNode XmlCommon::getParameterNode(QString pname, QDomElement& elem) {
    QDomNodeList n = elem.elementsByTagName(pname);
    return n.item(0);
}

QDomNode XmlCommon::getParameterNode(QString pname, QDomNode& node) {
    if(node.isNull())
        return node;
    QDomNodeList n = node.toElement().elementsByTagName(pname);
    return n.item(0);
}

QString XmlCommon::getParameterValue(QDomNode& node) {
    return node.toElement().attribute("value");
}

QString XmlCommon::getParameterValue(QDomElement& elem) {
    return elem.attribute("value");
}

QString XmlCommon::getParameterValue(QDomNode& node, QString name) {
    return node.toElement().attribute(name);
}

QString XmlCommon::getParameterValue(QDomElement& elem, QString name) {
    return elem.attribute(name);
}
