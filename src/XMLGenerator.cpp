#include "XMLGenerator.h"
#include "LearningConcept.h"
#include "HierarchyTree.h"

#include "tree.hh"

namespace miles {

QDomNode XMLTreeGenerator::getParameterNode(QString pname, QDomElement& elem) {
    QDomNodeList n = elem.elementsByTagName(pname);
    return n.item(0);
}

QDomNode XMLTreeGenerator::getParameterNode(QString pname, QDomNode& node) {
    if(node.isNull())
        return node;
    QDomNodeList n = node.toElement().elementsByTagName(pname);
    return n.item(0);
}

QString XMLTreeGenerator::getParameterValue(QDomNode& node) {
    return node.toElement().attribute("value");
}

QString XMLTreeGenerator::getParameterValue(QDomElement& elem) {
    return elem.attribute("value");
}



  XMLGenerator::XMLGenerator() {
    lineBuffer="";
  }
  
  XMLGenerator::~XMLGenerator() {
    endXML();
  }
  
  XMLGenerator::XMLGenerator(std::string fname) {
    startXML(fname);
    lineBuffer="";
  }
  
  void XMLGenerator::startXML(std::string fname){
    std::ifstream test;
    
    filename=fname;
    tablevel=0;
    // Test for file existence. If the file exists, it is removed.
    test.open(filename.c_str(), std::fstream::in);
    test.close();
    if(!(test.fail())) {
      remove(filename.c_str());
    } 
    
    file.open(filename.c_str(), std::fstream::out );
    if(file.is_open()) {
      file << "<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>\n";
    } else {
      printf("XMLGenerator: Error opening file.\n");
    }
  }
  
  void XMLGenerator::endXML(){
    file.close();
  }
  
  void XMLGenerator::appendBuffer(){
    if (lineBuffer.length()!=0) {
      file << lineBuffer << std::endl;
      lineBuffer="";
    }
  }
  
  bool XMLGenerator::emptyBuffer(){
    if (lineBuffer.length()==0)
      return true;
    return false;
  }
  
  
  void XMLGenerator::openTag(std::string tagname, std::list<std::string> attributes){
    if(!file.is_open()) {
      printf("XMLGenerator: Error writing in file.\n");
      return;
    }
    
    multipleInsert( tablevel, "\t");
    writeToBuffer("<" + tagname);
    if(!attributes.empty()) {
      std::list<std::string>::const_iterator iter;
      for(iter=attributes.begin();iter!=attributes.end(); iter++) {
	if(bufferExhausted((*iter)))
	  appendBuffer();
	if(emptyBuffer()) {
	  multipleInsert( tablevel, "\t");
	  multipleInsert(tagname.length() + 1, " ");
	} else
	  writeToBuffer(" "); 
	writeToBuffer((*iter));
      }
    }
    writeToBuffer(">");
    appendBuffer();
    tablevel++;
    openedTags.push_front(tagname);
    
  }
  
  void XMLGenerator::closeCurrentTag(){
    if(!openedTags.empty()) {
      multipleInsert( --tablevel, "\t");
      writeToBuffer("</" + openedTags.front() + ">");
      openedTags.pop_front();
      appendBuffer();
    }
  }
  
  void XMLGenerator::multipleInsert(int times, std::string input){
    int i;
    
    for(i=0;i<times;i++)
      lineBuffer += input;
    
  }
  
  void XMLGenerator::oneLineTag(std::string name, std::list<std::string> attributes) {
    if(!file.is_open()) {
      printf("XMLGenerator: Error writing in file.\n");
      return;
    }    
    multipleInsert( tablevel, "\t");
    writeToBuffer("<" + name);
    if(!attributes.empty()) {
      std::list<std::string>::const_iterator iter;
      for(iter=attributes.begin();iter!=attributes.end(); iter++) {
	if(bufferExhausted((*iter)))
	  appendBuffer();
	if(emptyBuffer()) {
	  multipleInsert( tablevel, "\t");
	  multipleInsert(name.length() + 1, " ");
	} else
	  writeToBuffer(" "); 
	writeToBuffer((*iter));
      }
    }
    writeToBuffer("/>");
    appendBuffer();
  }

  void XMLGenerator::closeTag(std::string tagName) { 
    bool remaining=true;
    while((remaining==true)||(!openedTags.empty())) {
      if(openedTags.front()==tagName)
	remaining=false;
      multipleInsert( --tablevel, "\t");
      writeToBuffer("</" + openedTags.front() + ">");
      openedTags.pop_front();
      appendBuffer();
    }
  }
  
  void XMLGenerator::writeToBuffer(std::string input) {
    lineBuffer += input;
  }
  
  bool XMLGenerator::bufferExhausted(std::string input) {
    if (lineBuffer.length() + input.length() >= MAX_LINE)
      return true;
    
  return false;
  }
  
  XMLTreeGenerator::XMLTreeGenerator(): XMLGenerator() {}
  XMLTreeGenerator::XMLTreeGenerator(std::string fname): XMLGenerator(fname) {}
  
  
  void XMLTreeGenerator::printNode(SpLearningStateConcept node, int parent, SpLearningAttributes availableAttributes) {
    
    std::list<std::string> attributes;
    char value[100];  
    std::string toString;
    
    sprintf(value,"%d",parent);
    toString=value;
    attributes.push_back("PARENT=\"" + toString + "\"");
    
    sprintf(value,"%d",(int)node->getID());
    toString=value;
    attributes.push_back("ID=\"" + toString + "\"");
    
    sprintf(value,"%.4f",node->getP());
    toString=value;
    attributes.push_back("P=\"" + toString + "\"");
    
    sprintf(value,"%d",node->getN());
    toString=value;
    attributes.push_back("N=\"" + toString + "\"");

    sprintf(value,"%d",node->NE);
    toString=value;
    attributes.push_back("NE=\"" + toString + "\"");


    openTag("STATE", attributes);

    int i, I;
  
    if( (I = node->numericalAttributeList.size()) > 0) {
      std::map <attribute_id, SpNumericalAttributeValuePair>::iterator numericalIt = node->numericalAttributeList.begin();
      for(i = 0; i < I; i++, numericalIt++) {
	attributes.clear();
	sprintf(value,"%s", availableAttributes->nameFromId[(*numericalIt).first].c_str());
	toString=value;
	attributes.push_back("NAME=\"" + toString + "\"");
	attributes.push_back("TYPE=\"NUMERIC\"");
	sprintf(value,"%.4f", (*numericalIt).second->getMean());
	toString=value;
	attributes.push_back("MEAN=\"" + toString + "\"");
	sprintf(value,"%.4f", (*numericalIt).second->getSigma());
	toString=value;
	attributes.push_back("SIGMA=\"" + toString + "\"");
	sprintf(value,"%.4f", (*numericalIt).second->RSum);
	toString=value;
	attributes.push_back("RSUM=\"" + toString + "\"");
	oneLineTag("ATTRIBUTE", attributes);
      }
    }

    if( (I = node->symbolicAttributeList.size()) > 0) {
      std::map <attribute_id, SpSymbolicAttributeValuePair>::iterator symbolicIt = node->symbolicAttributeList.begin();
      int j, m;
      std::map<std::string, SpSymbolicValue>::iterator values;
      SpSymbolicAttributeValuePair currentSym;	

      for(i = 0; i < I; i++, symbolicIt++) {
	currentSym = (*symbolicIt).second;

	attributes.clear();
	sprintf(value,"%s", availableAttributes->nameFromId[(*symbolicIt).first].c_str());
	toString=value;
	attributes.push_back("NAME=\"" + toString + "\"");
	attributes.push_back("TYPE=\"SYMBOLIC\"");
	sprintf(value,"%.4f", currentSym->RSum);
	toString=value;
	attributes.push_back("RSUM=\"" + toString + "\"");
	openTag("ATTRIBUTE", attributes);

	values = currentSym->values.begin();
	m = currentSym->values.size();
	
	for(j = 0; j < m; j++, values++) {
	  attributes.clear();
	  sprintf(value,"%s", (*values).first.c_str());
	  toString=value;
	  attributes.push_back("NAME=\"" + toString + "\"");
	  sprintf(value,"%d", (*values).second->N);
	  toString=value;
	  attributes.push_back("N=\"" + toString + "\"");
	  sprintf(value,"%.4f", (*values).second->RSum);
	  toString=value;
	  attributes.push_back("RSUM=\"" + toString + "\"");
	  oneLineTag("VALUE", attributes);	  
	}

	closeCurrentTag();
      }	
    }
  
    closeCurrentTag();
  }


  void XMLTreeGenerator::generateTree(SpHierarchyTree hierarchy, SpLearningAttributes availableAttributes) {

    tree<SpLearningStateConcept> *htree=hierarchy->getHierarchy();
    tree<SpLearningStateConcept>::iterator iter;
    tree<SpLearningStateConcept>::fixed_depth_iterator parentsIterator;
    tree<SpLearningStateConcept>::fixed_depth_iterator kidsIterator;
    int level, numberOfLevels=hierarchy->getDepth();
    std::list<std::string> attributes;

    openTag("TREE", attributes);
    if(htree->empty()) {
      openTag("STATES", attributes);
      closeCurrentTag();
      openTag("EVENTS", attributes);
      closeCurrentTag();
      closeCurrentTag();
      endXML();
      return;
    }
    openTag("STATES", attributes);

    openTag("LEVEL", attributes);
    
    printNode(*(htree->begin()), -1, availableAttributes);

    closeCurrentTag();

    if(numberOfLevels==0) {    
      closeCurrentTag();
      closeCurrentTag();
      endXML();
      return;
    }

    iter=htree->begin();
    
    parentsIterator=htree->begin_fixed(iter, 0);
  
    for(level=1;level<=numberOfLevels;level++) {
      openTag("LEVEL", attributes);    
      
      for(kidsIterator=htree->begin_fixed(iter, level); htree->is_valid(kidsIterator); kidsIterator++) {
	printNode((*kidsIterator), (*kidsIterator)->getParent(), availableAttributes);
      }
    
      closeCurrentTag();    
    }
    
    //TAG States
    closeCurrentTag();    
    
    openTag("EVENTS", attributes);

    char value[100];  
    std::string toString;
    long int s1, s2, e;
    double P, meanT, sigmaT, minT, maxT;
    int i, N, n = hierarchy->registeredEvents.size();
    std::map<long int, SpLearningEventConcept>::iterator eventsIt =  hierarchy->registeredEvents.begin();
    SpLearningEventConcept event;
    for(i=0;i<n;i++,eventsIt++) {
      attributes.clear();
      e = (*eventsIt).first;
      event = (*eventsIt).second;
      s1 = event->startingState;
      s2 = event->arrivingState;
      N = event->N;
      meanT = event->getMean();
      sigmaT = event->getSigma();
      P = event->P;
      minT = event->minT;
      maxT = event->maxT;

      sprintf(value,"%d",(int)e);
      toString=value;
      attributes.push_back("ID=\"" + toString + "\"");

      sprintf(value,"%d",(int)s1);
      toString=value;
      attributes.push_back("S1=\"" + toString + "\"");

      sprintf(value,"%d",(int)s2);
      toString=value;
      attributes.push_back("S2=\"" + toString + "\"");

      sprintf(value,"%d",N);
      toString=value;
      attributes.push_back("N=\"" + toString + "\"");
    
      sprintf(value,"%.4f",P);
      toString=value;
      attributes.push_back("P=\"" + toString + "\"");

      sprintf(value,"%.4f",meanT);
      toString=value;
      attributes.push_back("MEAN_T=\"" + toString + "\"");

      sprintf(value,"%.4f",sigmaT);
      toString=value;
      attributes.push_back("SIGMA_T=\"" + toString + "\"");

      sprintf(value,"%.4f",minT);
      toString=value;
      attributes.push_back("MIN_T=\"" + toString + "\"");

      sprintf(value,"%.4f",maxT);
      toString=value;
      attributes.push_back("MAX_T=\"" + toString + "\"");
      
      oneLineTag("EVENT", attributes);
	
    }

    //TAG Events
    closeCurrentTag();    
  
    closeTag("TREE");
    endXML();

  }
  
}
