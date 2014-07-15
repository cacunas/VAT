#ifndef XMLGENERATOR_H
#define XMLGENERATOR_H

#include <string>
#include <list>
#include <fstream>
#include "LearningAttributesSpecification.h"
#include <QDomNode>

#define MAX_LINE 256

namespace miles {

  class XMLGenerator {
  public:
    XMLGenerator();
    XMLGenerator(std::string);
    ~XMLGenerator();
    void startXML(std::string);
    void endXML();
    void openTag(std::string, std::list<std::string>);
    void oneLineTag(std::string, std::list<std::string>);
    void closeCurrentTag();
    void closeTag(std::string);
    
  private:
    std::ofstream file;
    std::string lineBuffer;
    std::list<std::string> openedTags;
    std::string filename;
    int tablevel;
    
    void writeToBuffer(std::string);
    void appendBuffer();
    void multipleInsert(int, std::string);
    bool bufferExhausted(std::string);
    bool emptyBuffer();
  };
  
  class XMLTreeGenerator: public XMLGenerator {
    
  public:
    XMLTreeGenerator();
    XMLTreeGenerator(std::string);
    
    void generateTree(SpHierarchyTree, SpLearningAttributes);
    void printNode(SpLearningStateConcept, int, SpLearningAttributes);
    static QDomNode getParameterNode(QString pname, QDomElement& elem);
    static QDomNode getParameterNode(QString pname, QDomNode& node);
    static QString getParameterValue(QDomNode& node);
    static QString getParameterValue(QDomElement& elem);

  };

  
}

#endif
