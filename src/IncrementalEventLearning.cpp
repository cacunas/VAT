#include <QSharedPointer>

#include "IncrementalEventLearning.h"
#include "VideoAnalysis.h"
#include <QFile>
#include <QTextStream>
#include<iostream>

#include "LearningContextualisedObject.h"
#include "XMLGenerator.h"
#include "LearningContext.h"
#include "HierarchyTree.h"
#include "LearningConcept.h"

namespace miles {

IncrementalEventLearning::IncrementalEventLearning(Datapool *i_data):m_data(i_data),nValidContexts(0) {xmlConfig = NULL;}
IncrementalEventLearning::~IncrementalEventLearning() {}

void IncrementalEventLearning::setContextualisedObjects(Datapool &datapool) {
#ifdef USE3D_MOBILES
    std::deque<SpMobileObject> *objectsList =  &(datapool.objects);
    std::deque<SpMobileObject>::iterator objectsIt = objectsList->begin();
    ObjectType object_type;
#else
    std::deque<SpMObject> *objectsList =  &(datapool.RMMobjects);
    std::deque<SpMObject>::iterator objectsIt = objectsList->begin();
#endif
    std::map<int, SpLearningContext>::iterator contextsIt;
    std::map<unsigned long, SpLearningContextualisedObject>::iterator followedIt = followedObjects.begin();
    int i, n = objectsList->size(), j, m = validContexts.size(), k, l = followedObjects.size();
    unsigned long object_id;
    std::map<int, SpLearningContext> validContextsForObject;
    std::set<ObjectType> *involved;

    //Initialise the flags for knowing if the object is lost for this frame
    presentInThisFrame.clear();
    for(k = 0; k < l; k++, followedIt++) {
        presentInThisFrame[(*followedIt).first] = false;
        (*followedIt).second->numFrames++;
    }
    for(i = 0; i < n; i++, objectsIt++) {
        object_id = (*objectsIt)->getMobileId();
        if(followedObjects.count(object_id) == 0) { //New contextualised object
            validContextsForObject.clear();
#ifdef USE3D_MOBILES
            if( (object_type = (*objectsIt)->getBestType() ) != UNKNOWN) { //If object is well classified
                contextsIt = validContexts.begin();
                for(j = 0; j < m; j++, contextsIt++) {
                    involved = &((*contextsIt).second->involvedObjects);
                    if( involved->count(ANY2D) == 1 || involved->count(ANY3D) == 1 || involved->count(object_type) == 1) {
                        validContextsForObject[(*contextsIt).first] = (*contextsIt).second;
                    }
                }
            } else {
                contextsIt = validContexts.begin();
                for(j = 0; j < m; j++, contextsIt++) {
                    involved = &((*contextsIt).second->involvedObjects);
                    if( involved->count(ANY2D) == 1)
                        validContextsForObject[(*contextsIt).first] = (*contextsIt).second;
                }
            }
#else
            contextsIt = validContexts.begin();
            for(j = 0; j < m; j++, contextsIt++) {
                involved = &((*contextsIt).second->involvedObjects);
                if( involved->count(ANY2D) == 1)
                    validContextsForObject[(*contextsIt).first] = (*contextsIt).second;
            }
#endif
            //If there is at least one valid learning context, process it
            if(validContextsForObject.size() > 0) {
                SpLearningContextualisedObject newContextualised(new LearningContextualisedObject(object_id));
                m = validContextsForObject.size();
                contextsIt = validContextsForObject.begin();
                for(j = 0; j < m; j++, contextsIt++) {
                    newContextualised->activeContexts.push_back((*contextsIt).first);
                    newContextualised->processLearningContext((*objectsIt), (*contextsIt).first, (*contextsIt).second, availableAttributes, datapool);
                    ((*contextsIt).second)->contextualisedObjects[object_id] = newContextualised;
                }
                followedObjects[object_id] = newContextualised;
                presentInThisFrame[object_id] = true;
                newContextualised->lastUpdatedFrame = datapool.frameNumber;
                newContextualised->numLostFrames = 0;
                newContextualised->numFrames = 1;
            }
        } else {
            //For already followed objects mark immediatelly as present
            presentInThisFrame[object_id] = true;
            followedObjects[object_id]->numLostFrames = 0;
            followedObjects[object_id]->updateConcernedLearningContexts((*objectsIt), availableAttributes, datapool);
            followedObjects[object_id]->lastUpdatedFrame = datapool.frameNumber;
        }
    }

    //Now for objects not found, update the counter of objects lost
    std::map<unsigned long, bool>::iterator isPresentIt = presentInThisFrame.begin();
    std::deque<unsigned long> toErase;
    n = presentInThisFrame.size();
    for(i = 0; i < n; i++, isPresentIt++) {
        if((*isPresentIt).second == false) {
            followedObjects[(*isPresentIt).first]->numLostFrames++;
            if(followedObjects[(*isPresentIt).first]->numLostFrames >= 10) //Pay attention here
                toErase.push_back((*isPresentIt).first);
        }
    }

    //Erase marked elements
    n = toErase.size();
    if(n > 0) {
        std::deque<unsigned long>::iterator toEraseIt = toErase.begin();
        std::deque<int>::iterator lcIt;
        SpLearningContextualisedObject toErase;
        SpLearningContext toCheck;
        for(i = 0; i < n; i++, toEraseIt++) {
            //Erase object reference from context
            toErase = followedObjects[(*toEraseIt)];
            lcIt = toErase->activeContexts.begin();
            m = toErase->activeContexts.size();
            for(j = 0; j < m; j++, lcIt++) {
                toCheck = validContexts[(*lcIt)];
                if(toCheck->contextualisedObjects.count((*toEraseIt)) > 0)
                    toCheck->contextualisedObjects.erase((*toEraseIt));
            }
            presentInThisFrame.erase(*toEraseIt);
            followedObjects.erase(*toEraseIt);
        }
    }
    
}

void IncrementalEventLearning::run(double tdiff) {
    //For time, if present make everything, if not, update permanence for the object in the current state
    std::map<unsigned long, bool>::iterator isPresentIt = presentInThisFrame.begin();
    std::deque<unsigned long> toErase;
    std::deque<int>::iterator activeContextsIt;
    SpLearningContextualisedObject currentObject;
    SpLearningContext currentContext;
    
    int i, n = presentInThisFrame.size(), j, m;
    for(i = 0; i < n; i++, isPresentIt++) {
      currentObject = followedObjects[(*isPresentIt).first];
      m = currentObject->activeContexts.size();
      activeContextsIt = currentObject->activeContexts.begin();

      if((*isPresentIt).second == true) {
	
	for(j = 0; j < m; j++, activeContextsIt++) {
	  currentContext = validContexts[(*activeContextsIt)];
          currentContext->eventsHierarchy->execute(currentContext, currentObject, tdiff);
	}
      } else { //If not present update, staying time at states
	int k, l;
        std::map<int, SpLevelData> *level;
        std::map<int, SpLevelData>::iterator levelIt;
	for(j = 0; j < m; j++, activeContextsIt++) {
	  level  = &currentObject->levelData[(*activeContextsIt)];
	  levelIt = level->begin();
	  l = level->size();
	  for(k = 0; k < l; k++, levelIt++)
	    (*levelIt).second->timeStayingAtCurrent += tdiff;
	}
      }
    }

    if(m_activateStats && validContexts.size() > 0) {
        if (!m_statsFile.open(QIODevice::Append | QIODevice::Text)) {
            AppendToLog("IncrementalEventLearning Error: Unable to open file '"+ m_hierarchyStatsFile +"' for append.");
            return;
        }
        SpHierarchyTree tree = validContexts[0]->eventsHierarchy;
        QTextStream statsStream(&m_statsFile);
        statsStream << m_data->frameNumber << "\t"
                    << (int)tree->registeredStates.size() << "\t"
                    << (int)tree->registeredEvents.size() << endl;
        m_statsFile.close();
    }

  }

  void IncrementalEventLearning::init() {
    SpLearningAttributes newAvailableAttributes(new LearningAttributes());
    availableAttributes = newAvailableAttributes;
    availableAttributes->setNumericalAttributes();
    availableAttributes->setSymbolicAttributes();
    HierarchyTree::availableAttributes = availableAttributes;
    SpXMLTreeGenerator newGenerator(new XMLTreeGenerator());
    treeGenerator = newGenerator;
  }

bool IncrementalEventLearning::setParameters(QDomNode &config) {

    QDomNode n,m;
    bool ok = false;
    if(config.isNull()) { //Parameter set for module not defined
          m_learningContextsFile = "config/defaultContextsFile.xml";
          m_activateStats = false;
    } else {
        if( ( n = XmlCommon::getParameterNode("ContextsFile", config) ).isNull() ) {
            m_learningContextsFile = "config/defaultContextsFile.xml";
            AppendToLog(QString("IncrementalEventLearning: Warning: Parameter ContextsFile not defined. \n")+
                        QString("\t\tTrying to use default file:\t config/defaultContextsFile.xml ..."));
        } else {
            m_learningContextsFile = XmlCommon::getParameterValue(n);
            QFile file(m_learningContextsFile);
            if (!file.open(QIODevice::ReadOnly)) {
              AppendToLog("\nIncrementalEventLearning: Error: 'Cannot open file :\t " + m_learningContextsFile +
                          " .\n\tTrying to use default file:\t config/defaultContextsFile.xml ...");
            } else {
                ok = true;
                file.close();
            }
        }

        if( ( n = XmlCommon::getParameterNode("ActivateHierarchyStats", config) ).isNull() ) {
            m_activateStats = false;
        } else {
            m_activateStats = (XmlCommon::getParameterValue(n)=="yes") ? true : false;
            if(m_activateStats) {
                if( ( m = XmlCommon::getParameterNode("OutputFileName", n) ).isNull() ) {
                    m_hierarchyStatsFile = "miles_results.txt";
                } else {
                    m_hierarchyStatsFile = XmlCommon::getParameterValue(m);
                    if(m_hierarchyStatsFile == "")
                        m_hierarchyStatsFile = "miles_results.txt";
                }
                m_statsFile.setFileName(m_hierarchyStatsFile);
                //Erasing file and checking write permissions
                if (!m_statsFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
                    AppendToLog("IncrementalEventLearning Error: Unable to open file '" + m_hierarchyStatsFile +"' for writing.");
                } else {
                    QTextStream statsStream(&m_statsFile);
                    statsStream << "Frame\tStates\tEvents" <<endl;
                    m_statsFile.close();
                }
            }
        }
    }

    if(ok == false) {
        QFile file(m_learningContextsFile);
        if (!file.open(QIODevice::ReadOnly)) {
            AppendToLog("\nIncrementalEventLearning: Error: 'Cannot open file :\t config/defaultContextsFile.xml \n Aborting as now no Learning Contexts are defined for Event Learning.\n\n");
            return false;
        }
        file.close();
    }
    return true;
}


  bool IncrementalEventLearning::invalidNumericNameForContext(std::string name, SpLearningContext context) {
    if(availableAttributes->idFromName.count(name) == 0)
      return true;
    attribute_id att_id = availableAttributes->idFromName[name];
    if(context->numericalAttributes.count(att_id) == 0)
      return true;
    return false;
  }

  bool IncrementalEventLearning::invalidSymbolicNameForContext(std::string name, SpLearningContext context) {
    if(availableAttributes->idFromName.count(name) == 0)
      return true;
    attribute_id att_id= availableAttributes->idFromName[name];
    if(context->symbolicAttributes.count(att_id) == 0)
      return true;
    return false;
  }
  bool IncrementalEventLearning::invalidValueNameForContext(attribute_id att_id, std::string name) {
    if ( (*availableAttributes->symbolicValuesList[att_id]).count(name) == 0 )
      return true;
    return false;
  }

  SpLearningStateConcept IncrementalEventLearning::createNewNode(QDomNode& node, SpLearningContext context, long int &free_state_id) {
    SpLearningStateConcept newNode(new LearningStateConcept());

    int i, n, j, m, N;
    attribute_id att_id;
    double mean, sigma, RSum, vRSum;
    std::map<std::string, SpSymbolicValue> values;
    if(node.isNull())
        return newNode;
    QDomNode nn = XMLTreeGenerator::getParameterNode("PARENT", node);
    newNode->setParent(XMLTreeGenerator::getParameterValue(nn).toInt());
    nn = XMLTreeGenerator::getParameterNode("ID", node);
    newNode->setID(XMLTreeGenerator::getParameterValue(nn).toInt());
    nn = XMLTreeGenerator::getParameterNode("P", node);
    newNode->setP(XMLTreeGenerator::getParameterValue(nn).toDouble());
    nn = XMLTreeGenerator::getParameterNode("N", node);
    newNode->setN(XMLTreeGenerator::getParameterValue(nn).toInt());
    nn = XMLTreeGenerator::getParameterNode("NE", node);
    newNode->NE = XMLTreeGenerator::getParameterValue(nn).toInt();

    if(newNode->id > free_state_id)
      free_state_id = newNode->id;
    nn = XMLTreeGenerator::getParameterNode("ATTRIBUTE", node);
    QDomNodeList nl = nn.childNodes();
    QDomNode child_node, att_node, val_node;
    QString att, val;
    std::string auxString, auxString2;
    n = nl.length();
    if(n == 0) {
      std::cout << "IncrementalEventLearning: Error: No ATTRIBUTE entry found. Aborting...\n\n";
      return newNode;
    }

    for(i = 0; i < n; i++) {
      child_node = nl.at(i);
      att_node = XMLTreeGenerator::getParameterNode("TYPE", child_node);
      att = XMLTreeGenerator::getParameterValue(att_node);
      if(att == "NUMERIC") {
        val_node  = XMLTreeGenerator::getParameterNode("NAME", child_node);
        val = XMLTreeGenerator::getParameterValue(val_node);
        auxString = val.toStdString();
	if(invalidNumericNameForContext(auxString, context)) {
          std::cout << "IncrementalEventLearning: Error: Numerical attribute "
		    << auxString <<" not found for context "
            << context->name.toStdString() << ". Aborting...\n\n";
          return newNode;
	}
	att_id = availableAttributes->idFromName[auxString];
        val_node  = XMLTreeGenerator::getParameterNode("MEAN", child_node);
        mean = XMLTreeGenerator::getParameterValue(val_node).toDouble();
        val_node  = XMLTreeGenerator::getParameterNode("SIGMA", child_node);
        sigma = XMLTreeGenerator::getParameterValue(val_node).toDouble();
        val_node  = XMLTreeGenerator::getParameterNode("RSUM", child_node);
        RSum = XMLTreeGenerator::getParameterValue(val_node).toDouble();
	newNode->insertNumericalPair(att_id, mean, sigma, RSum);
      } else { //SYMBOLIC
        val_node  = XMLTreeGenerator::getParameterNode("NAME", child_node);
        val = XMLTreeGenerator::getParameterValue(val_node);
        auxString = val.toStdString();
	if(invalidSymbolicNameForContext(auxString, context)) {
          std::cout << "IncrementalEventLearning: Error: Numerical attribute "
		    << auxString <<" not found for context "
            << context->name.toStdString() << ". Aborting...\n\n";
          return newNode;
	}
	att_id = availableAttributes->idFromName[auxString];
        val_node  = XMLTreeGenerator::getParameterNode("RSUM", child_node);
        RSum = XMLTreeGenerator::getParameterValue(val_node).toDouble();
	
        QDomNodeList ml = XMLTreeGenerator::getParameterNode("VALUE",child_node).childNodes();
        m = ml.length();
	if(m == 0) {
          std::cout << "IncrementalEventLearning: Error: No VALUE entries found for symbolic attribute "
		    << auxString << ". Aborting...\n\n";
          return newNode;
	}
	values.clear();
        for(j = 0; j < m; j++) {
            QDomNode value_node = ml.at(i);
            val_node  = XMLTreeGenerator::getParameterNode("NAME", value_node);
            val = XMLTreeGenerator::getParameterValue(val_node);
            auxString2 = val.toStdString();
	  if(invalidValueNameForContext(att_id, auxString)) {
            std::cout << "IncrementalEventLearning: Error: Value "
		      << auxString <<" not found for symbolic attribute "
                      << auxString2 << ". Aborting...\n\n";
            return newNode;
	  }
          val_node  = XMLTreeGenerator::getParameterNode("N", value_node);
          N = XMLTreeGenerator::getParameterValue(val_node).toInt();
          val_node  = XMLTreeGenerator::getParameterNode("RSUM", value_node);
          vRSum = XMLTreeGenerator::getParameterValue(val_node).toDouble();
	  SpSymbolicValue newValue( new SymbolicValue(vRSum, vRSum/RSum, N) );
	  values[auxString] = newValue;
	}
	newNode->insertSymbolicPair(att_id, RSum, &values);
      }

    }

    return newNode;
  
  }


  SpHierarchyTree IncrementalEventLearning::generateTree(QString filename, SpLearningContext context) {

    SpHierarchyTree loadedTree(new HierarchyTree());
    tree <SpLearningStateConcept> *hierarchy = loadedTree->getHierarchy();
    tree<SpLearningStateConcept>::iterator Node=hierarchy->begin();
    tree<SpLearningStateConcept>::fixed_depth_iterator parent_iter;
    tree<SpLearningStateConcept>::iterator new_iter;

    int i, j, levels=0;
    int currentParent, numLevels, numNodes;
    long int free_state_id = 0, free_event_id = 0;

    if(xmlConfig != NULL)
        delete xmlConfig;
    QDomDocument *xmlConfig = new QDomDocument( "TREE" );
    QFile file( filename );
    if( !file.open(QIODevice::ReadOnly) ) {
        std::cout << "IncrementalEventLearning: Error: Error opening file '"
                  << filename.toStdString()
                  << "' for reading. Execution will be aborted." << std::endl;
        return loadedTree;
    }

    QString error;
    int line, column;
    if( !xmlConfig->setContent( &file, &error, &line, &column ) ) {

        std::cout << "IncrementalEventLearning: Error: Error opening configuration file '"
                     << filename.toStdString() << "'. XML content could not be extracted. Execution will be aborted.\n"
                     << "Error on line " << QString::number(line).toStdString() << " and column "
                     << QString::number(column).toStdString() << ": " << error.toStdString() << std::endl;
        file.close();
        return loadedTree;
    }
    file.close();

    rootConfig = xmlConfig->documentElement();
    if( rootConfig.tagName() != "TREE" ) {
        std::cout << "IncrementalEventLearning: Error: No TREE tag found. Aborting...\n\n";
        return loadedTree;
    }

    QDomNode n, m, val;
    QDomNodeList l1, l2;

    n = rootConfig.elementsByTagName("STATES").item(0);
    if( n.isNull() ) {
        std::cout << "IncrementalEventLearning: Error: No STATES tag found. Aborting...\n\n";
        return loadedTree;
    }

    l1 = n.toElement().elementsByTagName("LEVEL");
    numLevels = l1.length();
    if(numLevels == 0) {
      std::cout << "IncrementalEventLearning: Error: No LEVEL entry found. Aborting...\n\n";
      return loadedTree;
    }

    //For root level
    l2 = l1.at(0).toElement().elementsByTagName("STATE");
    numNodes = l2.length();
    if(numNodes > 1) {
      std::cout << "IncrementalEventLearning: Error: Too many STATES for first level. Aborting...\n\n";
      return loadedTree;
    }
    m = l2.at(0);
    hierarchy->insert(Node, createNewNode(m, context, free_state_id));
    Node = hierarchy->begin();
    loadedTree->registeredStates[(*Node)->id] = (*Node);
    levels=0;    

    if(numLevels > 1)
      for(i = 1; i < numLevels; i++) {
	levels++;
        l2 = l1.at(i).toElement().elementsByTagName("STATE");
        numNodes = l2.length();
	if(numNodes == 0) {
          std::cout << "IncrementalEventLearning: Error: No STATE entry found. Aborting...\n\n";
          return loadedTree;
	}

        for(j = 0; j < numNodes; j++) {
            m = l2.at(j);
            parent_iter=hierarchy->begin_fixed(Node, levels - 1);
     
            val = XMLTreeGenerator::getParameterNode("PARENT",m);
            currentParent = XMLTreeGenerator::getParameterValue(val).toInt();

            while(currentParent != (*parent_iter)->getID())
                parent_iter++;

            new_iter = hierarchy->append_child(parent_iter, createNewNode(m, context, free_state_id));
            loadedTree->registeredStates[(*new_iter)->id] = (*new_iter);
	}

      }



    n = rootConfig.elementsByTagName("EVENTS").item(0);
    if( n.isNull() ) {
        std::cout << "IncrementalEventLearning: Error: No EVENTS tag found. Aborting...\n\n";
        return loadedTree;
    }

    l1 = n.toElement().elementsByTagName("EVENT");
    numNodes = l1.length();

    long int id;
    int N;
    double P, meanT, sigmaT, minT, maxT;
    long int s1, s2;
    for(j = 0; j < numNodes; j++) {
        m = l1.at(j);
        val = XMLTreeGenerator::getParameterNode("ID",m);
        id = XMLTreeGenerator::getParameterValue(val).toInt();
        val = XMLTreeGenerator::getParameterNode("S1",m);
        s1 = XMLTreeGenerator::getParameterValue(val).toInt();
        val = XMLTreeGenerator::getParameterNode("S2",m);
        s2 = XMLTreeGenerator::getParameterValue(val).toInt();
        val = XMLTreeGenerator::getParameterNode("N",m);
        N = XMLTreeGenerator::getParameterValue(val).toInt();
        val = XMLTreeGenerator::getParameterNode("P",m);
        P = XMLTreeGenerator::getParameterValue(val).toDouble();
        val = XMLTreeGenerator::getParameterNode("MEAN_T",m);
        meanT = XMLTreeGenerator::getParameterValue(val).toDouble();
        val = XMLTreeGenerator::getParameterNode("SIGMA_T",m);
        sigmaT = XMLTreeGenerator::getParameterValue(val).toDouble();
        val = XMLTreeGenerator::getParameterNode("MIN_T",m);
        minT = XMLTreeGenerator::getParameterValue(val).toDouble();
        val = XMLTreeGenerator::getParameterNode("MAX_T",m);
        maxT = XMLTreeGenerator::getParameterValue(val).toDouble();

        if(id > free_event_id)
            free_event_id = id;

        SpLearningEventConcept newEvent(new LearningEventConcept(id, meanT, sigmaT));
        newEvent->N = N;
        newEvent->P = P;
        newEvent->minT = minT;
        newEvent->maxT = maxT;
        newEvent->startingState = s1;
        newEvent->arrivingState = s2;

        loadedTree->registeredEvents[id] = newEvent;
        loadedTree->registeredStates[s1]->outgoingEvents[s2] = newEvent;
        loadedTree->registeredStates[s2]->incomingEvents[s1] = newEvent;

    }

    free_state_id++;
    free_event_id++;

    loadedTree->free_event_id = free_event_id;
    loadedTree->free_state_id = free_state_id; 
    loadedTree->setDepth(levels);

    return loadedTree;
  }


  void IncrementalEventLearning::setLearningContexts() {
    int i, j, num, numContexts, numValid;
    attribute_id id;
    double acuity;
    std::string auxString;
    ObjectType currentType;
    QString str_val;

    QDomDocument config( "LEARNING_CONTEXT_LIST" );
    QDomElement root;
    QFile file( m_learningContextsFile );
    if( !file.open(QIODevice::ReadOnly) ) {
        std::cout << "IncrementalEventLearning: Error: Error opening file '"
                  << m_learningContextsFile.toStdString()
                  << "' for reading. Execution will be aborted." << std::endl;
        return;
    }

    QString error;
    int line, column;
    if( !config.setContent( &file, &error, &line, &column ) ) {

        std::cout << "IncrementalEventLearning: Error: Error opening configuration file '"
                     << m_learningContextsFile.toStdString() << "'. XML content could not be extracted. Execution will be aborted.\n"
                     << "Error on line " << QString::number(line).toStdString() << " and column "
                     << QString::number(column).toStdString() << ": " << error.toStdString() << std::endl;
        file.close();
        return;
    }
    file.close();

    root = config.documentElement();
    if( root.tagName() != "LEARNING_CONTEXT_LIST" ) {
        std::cout << "IncrementalEventLearning: Error: No LEARNING_CONTEXT_LIST tag found. Aborting...\n\n";
        return;
    }

    QDomNode n, m, val;
    QDomNodeList l1, l2, laux;

    //Process Learning Contexts
    l1 = root.elementsByTagName("Learning_Context");
    numContexts = l1.length();
    if(numContexts == 0) {
        std::cout << "IncrementalEventLearning: Error: No Learning_Context entry found. Aborting...\n\n";
        return;
    }

    this->nValidContexts = 0;
    for(i = 0; i < numContexts; i++) {
        n = l1.at(i);
        str_val = n.toElement().attribute("name");

        if(str_val == "") {
            str_val = "LearningContext" + QString::number(i);
            std::cout << "IncrementalEventLearning: Warning: Learning_Context name attribute not found. Changing name to "
                      << str_val.toStdString() << std::endl;
        }


        if(    n.toElement().elementsByTagName("Involved_Object_List").length() == 0
            || (n.toElement().elementsByTagName("Numerical_Attribute_List").length() == 0 &&
                n.toElement().elementsByTagName("Symbolic_Attribute_List").length() == 0)) {
            std::cout << "IncrementalEventLearning: Warning: Learning_Context of name "
                      << str_val.toStdString() << " does not define a Involved_Object_List entry or at least one of the entries"
                      << " Numerical_Attribute_List and Symbolic_Attribute_List. Skipping this learning context.\n";
            continue;
        }
        SpLearningContext newContext(new LearningContext());
        QString init_file, str_val2;
        newContext->name = str_val;
        init_file = n.toElement().attribute("init_file");
        newContext->initialHierarchyFile = init_file;

        //Now tags of learning context

        //Involved Objects

        laux = n.toElement().elementsByTagName("Involved_Object_List");
        if(laux.length() == 0) {
            std::cout << "IncrementalEventLearning: Warning: No Involved_Object_List entry found. Skipping learning context "
                  << newContext->name.toStdString() << "...\n" << std::endl;
            continue;
        }
        m = laux.at(0);
        l2 = m.toElement().elementsByTagName("Involved_Object");
        num = l2.length();
        if(num == 0) {
            std::cout << "IncrementalEventLearning: Warning: No Involved_Object entry found. Skipping learning context "
                  << newContext->name.toStdString() << "...\n" << std::endl;
            continue;
        }

        numValid = 0;
        for(j = 0; j < num; j++) {
            m = l2.at(j);            
            str_val2 = m.toElement().attribute("name");
            if(str_val2 == "") {
                std::cout << "IncrementalEventLearning: Warning: name attribute of Involved_Object is an empty string. Skipping this Involved_Object entry.\n";
                continue;
            }

            if( (currentType = Blob::getTypeFromName(str_val2.toStdString())) == UNKNOWN) {
                std::cout << "IncrementalEventLearning: Warning: Class Type of Involved_Object not supported. Skipping this Involved_Object entry." << std::endl;
                continue;
            }
            numValid++;
            newContext->involvedObjects.insert(currentType);
        }
        if(numValid == 0) {
            std::cout << "IncrementalEventLearning: Warning: No valid Involved_Object entry found. Skipping learning context "
                      << newContext->name.toStdString() << "...\n\n";
            continue;
        }

        //Set a unique value if ANY2D or ANY3D are present
        newContext->cleanInvolvedList();

        //Now the counter is for both numerical and symbolic attributes
        numValid = 0;

        //Numerical Attributes
        laux = n.toElement().elementsByTagName("Numerical_Attribute_List");
        /*if(laux.length() == 0) {
            std::cout << "IncrementalEventLearning: Warning: No Numerical_Attribute_List entry found. Skipping learning context "
                  << newContext->name << "...\n" << std::endl;
            continue;
        }*/
        m = laux.at(0);
        l2 = m.toElement().elementsByTagName("Numerical_Attribute");
        num = l2.length();
        if(num > 0) {
            for(j = 0; j < num; j++) {
                m = l2.at(j);
                str_val2 = m.toElement().attribute("name");
                if(str_val2 == "") {
                    std::cout << "IncrementalEventLearning: Warning: name attribute of Numerical_Attribute is an empty string. Skipping this Numerical_Attribute entry." << std::endl;
                    continue;
                }
                if( availableAttributes->availableNumerical.count(str_val2.toStdString()) == 0) {
                    std::cout << "IncrementalEventLearning: Warning: Numerical attribute "
                              << str_val2.toStdString() << " not available. Skipping this Numerical_Attribute entry.\n";
                    continue;
                }

                acuity = m.toElement().attribute("acuity").toDouble();
                if(acuity <= 0) {
                    std::cout << "IncrementalEventLearning: Warning: Invalid acuity value for numerical attribute "
                              << str_val2.toStdString() << ". Acuity value must be higher than zero. Taking default: "
                              << DEFAULT_ACUITY << std::endl;
                    acuity = DEFAULT_ACUITY;
                }

                id = availableAttributes->idFromName[str_val2.toStdString()];
                numValid++;
	  
                SpLearningNumeric newNumeric(new LearningNumeric(id, acuity));
                newContext->numericalAttributes[id] = newNumeric;
            }
        }

        //Now symbolic attributes
        laux = n.toElement().elementsByTagName("Symbolic_Attribute_List");
        if(laux.length() == 0) {
            if(newContext->numericalAttributes.size()==0) {
                std::cout << "IncrementalEventLearning: Warning: No Symbolic_Attribute_List entry found and no valid numerical attribute found either. Skipping learning context "
                          << newContext->name.toStdString() << "...\n" << std::endl;
                continue;
            }
        }
        m = laux.at(0);
        l2 = m.toElement().elementsByTagName("Symbolic_Attribute");
        num = l2.length();
        if(num > 0) {
            for(j = 0; j < num; j++) {
                m = l2.at(j);
                val = XMLTreeGenerator::getParameterNode("name", m);
                str_val2 = XMLTreeGenerator::getParameterValue(val);
                if(str_val2 == "") {
                    std::cout << "IncrementalEventLearning: Warning: name attribute of Symbolic_Attribute is an empty string. Skipping this Symbolic_Attribute entry.\n";
                    continue;
                }
                if( availableAttributes->availableSymbolic.count(str_val2.toStdString()) == 0 ) {
                    std::cout << "IncrementalEventLearning: Warning: Symbolic attribute "
                              << str_val2.toStdString() << " not available. Skipping this Symbolic_Attribute entry.\n";
                    continue;
                }

                id = availableAttributes->idFromName[str_val2.toStdString()];
                numValid++;
	  
                SpLearningSymbolic newSymbolic(new LearningSymbolic(id));
                newContext->symbolicAttributes[id] = newSymbolic;
            }
        }

        if(numValid == 0) {
            std::cout << "IncrementalEventLearning: Warning: No valid Attribute entry found. Skipping learning context "
                      << newContext->name.toStdString() << "...\n\n";
            continue;
        }
        //Insert the learning context and associated an id.
        newContext->id = this->nValidContexts;
        this->validContexts[this->nValidContexts] = newContext;
        this->nValidContexts++;

        //Set initial tree
        if(newContext->initialHierarchyFile != "")
            newContext->eventsHierarchy = generateTree(newContext->initialHierarchyFile, newContext);
        else {
            SpHierarchyTree newEventsHierarchy(new HierarchyTree());
            newContext->eventsHierarchy = newEventsHierarchy;
        }
    }

    if(this->nValidContexts > 0) {
        for(i = 0 ; i < this->nValidContexts; i++)
            validContexts[i]->print(availableAttributes);
    } else {
        std::cout << "IncrementalEventLearning: Error: No valid Learning Context found in file '"
                  << m_learningContextsFile.toStdString() <<"'. Aborting...\n\n";
        return;
    }

}


} // end namespace

