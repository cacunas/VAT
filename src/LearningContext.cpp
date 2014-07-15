#include "LearningContext.h"
#include "LearningConcept.h"
#include "HierarchyTree.h"
#include "VideoAnalysis.h"

#include <iostream>
#include <fstream>
#include <iomanip>

namespace miles {

  LearningNumeric::LearningNumeric(attribute_id i_att_id, double i_acuity):att_id(i_att_id), acuity(i_acuity) {}

  LearningNumeric::~LearningNumeric(){}

  LearningSymbolic::LearningSymbolic(attribute_id i_att_id):att_id(i_att_id) {}

  LearningSymbolic::~LearningSymbolic(){}

  LearningContext::LearningContext() {
      SpHierarchyTree nullOne;
            eventsHierarchy = nullOne;
  }

  LearningContext::~LearningContext() {}
  void LearningContext:: cleanInvolvedList() {
    int size;
    if ( (size = involvedObjects.size()) == 1)
      return;
    bool found3D = false;
    int i;
    std::set<ObjectType>::iterator involvedIt;

    for(i=0, involvedIt = involvedObjects.begin(); i<size; i++, involvedIt++) {
      if((*involvedIt) == ANY2D) {
	involvedObjects.clear();
        involvedObjects.insert(ANY2D);
	return;
      }
      if((*involvedIt) == ANY3D)
	found3D = true;
    }

    if(found3D) {
      involvedObjects.clear();
      involvedObjects.insert(ANY3D);
    }
  }

  void LearningContext:: print(SpLearningAttributes availableAttributes) {
    int i, j, n, m;
    std::set<std::string> *values;
    std::set<ObjectType>::iterator involvedIt;
    std::set<std::string>::iterator valuesIt;
    std::map<attribute_id, SpLearningNumeric>::iterator numIt;
    std::map<attribute_id, SpLearningSymbolic>::iterator symIt;

    AppendToLog("Learning Context: " + name);
    AppendToLog("\n    Initial Tree File: " + initialHierarchyFile);
    AppendToLog("\n    Involved Objects: \n");
    n = involvedObjects.size();
    for(i = 0, involvedIt = involvedObjects.begin(); i < n; i++, involvedIt++)
        AppendToLog("        " + QString(Blob::getNameFromType(*involvedIt).c_str()) + "\n");

    AppendToLog("    Numerical Attributes: \n");
    n = numericalAttributes.size();
    for(i = 0, numIt = numericalAttributes.begin(); i < n; i++, numIt++)
        AppendToLog("        " + QString(availableAttributes->nameFromId[(*numIt).first].c_str()) + "    Acuity: " + QString::number(((*numIt).second)->acuity) + "\n");

    AppendToLog("    Symbolic Attributes: \n");
    n = symbolicAttributes.size();
    for(i = 0, symIt = symbolicAttributes.begin(); i < n; i++, symIt++) {
        AppendToLog("        " + QString(availableAttributes->nameFromId[(*numIt).first].c_str()) + "\n");
        AppendToLog("        Values: \n");
        values = availableAttributes->symbolicValuesList[(*symIt).first];
        m = values->size();
        for(j = 0, valuesIt = values->begin(); j < m; j++, valuesIt++)
            AppendToLog("                " + QString((*valuesIt).c_str()) + "\n");
    }
  }

  void LearningContext::generateGnuPlot(std::string filename, int max_distance_to_leaf) {

    std::ifstream test;
    
    // Test for file existence. If the file exists, it is removed.
    test.open(filename.c_str(), std::fstream::in);
    test.close();
    if(!(test.fail())) {
      remove(filename.c_str());
    } 
    
    std::ofstream file;
    
    file.open(filename.c_str(), std::fstream::out );
    if(!file.is_open()) {
      std::cout << "Events-Visualiser: Error opening file for writing: '" << filename << "'. Aboting generation of gnuplot .plt file for tree.\n\n";
      return;
    } 
    
    file << "# Plot generator for learning context " << name.toStdString() << ".\n\n";
    
    //Write the general definitions here
    /*    file << "set terminal x11\nset parametric\nset trange [0:2*pi]\nset size 1,1\n"
	 << "set xlabel \"x [cm]\"\nset ylabel \"y [cm]\"\nunset title\n"
	 << "set object 1 rectangle from 710,330 to 740,230 fc rgb \"red\" fillstyle solid 1.0\n"
	 << "set mxtics 1\nset mytics 1\nset grid xtics ytics mxtics mytics\nset pointsize 2.0\n"
	 << "set key height 0.8 width 1 spacing 1 at 150,400 box lw 1\n\n";
    */
    bool graph3D = false;

    if(graph3D) {
      file << "set terminal x11" << std::endl;
      file << "set isosample 50,50" << std::endl;
      //file << "set pm3d" << std::endl;
      file << "set pm3d map at b" << std::endl;
      file << "set ticslevel 0.8" << std::endl;
      file << "set xrange [0:1000]" << std::endl;
      file << "set yrange [0:1000]" << std::endl;
      //file << "set parametric" << std::endl;
      file << "set size 0.84,1.05" << std::endl;
      file << "set xlabel \"x [cm]\"" << std::endl;
      file << "set ylabel \"y [cm]\"" << std::endl;
      file << "set cblabel \"P\"   " << std::endl;
      file << "unset title" << std::endl;
      file << "unset key" << std::endl;
      file << "set object 1 rectangle from 710,330 to 740,230 fc rgb \"red\" fillstyle solid 1.0 front" << std::endl << std::endl;
      file << "set mxtics 1" << std::endl; 
      file << "set mytics 1" << std::endl; 
      file << "set grid xtics ytics mxtics mytics" << std::endl; 
      file << "set pointsize 2.0" << std::endl; 
    } else {
      file << "set terminal x11  \"Times-Roman,12\" " << std::endl;
      file << "set parametric" << std::endl;
      file << "set trange [0:2*pi]" << std::endl;
      file << "set xtics nomirror" << std::endl;
      file << "set ytics nomirror" << std::endl;
      file << "set xlabel \"x [cm]\"" << std::endl;
      file << "set ylabel \"y [cm]\"" << std::endl;
      file << "set xrange [0:1000]" << std::endl;
      file << "set yrange [0:1000]" << std::endl;
      file << "set bmargin 5" << std::endl;
      file << "set tmargin 2" << std::endl;
      file << "unset title" << std::endl;
      file << "unset key" << std::endl;
      file << "set size 0.7,1.0" << std::endl;
      file << "set mxtics 1" << std::endl;
      file << "set mytics 1" << std::endl;
      file << "set grid xtics ytics mxtics mytics" << std::endl;
      file << "set pointsize 2.0" << std::endl;
      file << "set object 1 rectangle from 710,330,2 to 740,230,2 fc rgb \"red\" fillstyle solid 1.0 front" << std::endl;
      file << "set mxtics 1" << std::endl;
      file << "set mytics 1" << std::endl;
      file << "set grid xtics ytics mxtics mytics" << std::endl;
      file << "set pointsize 2.0" << std::endl;
    }
      
    int level=0;
    tree<SpLearningStateConcept>::iterator Node=eventsHierarchy->conceptsHierarchy.begin();
    
    if(eventsHierarchy->isEmptyTree(Node)) {
      file.close();
      return;
    }
    
    tree<SpLearningStateConcept>::fixed_depth_iterator level_iter;
    std::map<attribute_id, SpLearningNumeric>::iterator Num = numericalAttributes.begin();
    attribute_id att1 = (*Num++).first;
    attribute_id att2 = (*Num).first;
    double mean1, sigma1, mean2, sigma2;
    bool first = true;
    int kids;
    std::deque<int> nnodes;
    std::map<long int, SpLearningStateConcept> conceptsToShow;
    double maxP = 0.0;
    int numI =(*Node)->N;

    nnodes.push_back(1);
    //  file << "plot " << mean1 << " + " << sigma1 << "*cos(t),"  << mean2 << " + " << sigma2 << "*sin(t) title 'State " << (*Node)->id << "' lt 1 lw 1.5, \\\n"
    //     << mean1 << "+1*cos(t), "<< mean2 << "+1*sin(t) notitle lt 1"; 
    
    kids=eventsHierarchy->conceptsHierarchy.number_of_children(Node);
    if(kids>0)
      nnodes.push_back(kids);
      
    if(graph3D) {

      for(level=1;kids>0;level++) {
	//    file << "\n# For tree level " << level << ":\n";
	kids=0;
	for(level_iter=eventsHierarchy->conceptsHierarchy.begin_fixed(Node, level);level_iter!=0;level_iter++) {
	  kids+=eventsHierarchy->conceptsHierarchy.number_of_children(level_iter);
	  if((*level_iter)->distance_to_leaf <= max_distance_to_leaf) {
	    conceptsToShow[(*level_iter)->id] = (*level_iter);
	    if(maxP < (double)(*level_iter)->N / (double)numI )
	      maxP = (double)(*level_iter)->N / (double)numI;
	  }
	}
	if(kids > 0)
	  nnodes.push_back(kids);
      }

      file << "set zrange [0:" << maxP << "]" << std::endl;
      file << "set cbrange [0:" << maxP << "]" << std::endl;
      file << "set palette defined (0 \"white\", " << maxP << " \"black\")" << std::endl;

      std::map<long int, SpLearningStateConcept>::iterator conceptsToShowIt = conceptsToShow.begin();
      int i, n = conceptsToShow.size();
      
      for(i=0; i<n; i++, conceptsToShowIt++) {
	
	mean1 = (*conceptsToShowIt).second->numericalAttributeList[att1]->getMean();
	sigma1 = (*conceptsToShowIt).second->numericalAttributeList[att1]->getSigma();
	mean2 = (*conceptsToShowIt).second->numericalAttributeList[att2]->getMean();
	sigma2 = (*conceptsToShowIt).second->numericalAttributeList[att2]->getSigma();
	file << "f" << i << "(x,y) = " << (double)(*conceptsToShowIt).second->N / (double)numI
	     << "*(exp(-((" << mean1 << " - x)**2)/(2*" << sigma1
	     <<"**2)))*(exp(-((" << mean2 << " - y)**2)/(2*" << sigma2 << "**2)))\n" ;
      }
      
      conceptsToShowIt = conceptsToShow.begin();
      
      for(i=0; i<n; i++, conceptsToShowIt++) {
	if(first) {
	  first = false;
	  file << "g" << i << "(x,y) = f" << i << "(x,y)\n";  
	} else 
	  file << "g" << i << "(x,y) = f" << i << "(x,y) > g" << i-1 
	       << "(x,y) ? f" << i << "(x,y) : g" << i-1 << "(x,y) \n";  
      }
      
      file << "splot g" << n-1 << "(x,y) with pm3d\n\n";
      
    } else {
      
      for(level=1;kids>0;level++) {
	//    file << "\n# For tree level " << level << ":\n";
	kids=0;
	for(level_iter=eventsHierarchy->conceptsHierarchy.begin_fixed(Node, level);level_iter!=0;level_iter++) {
	  kids+=eventsHierarchy->conceptsHierarchy.number_of_children(level_iter);
	  if(eventsHierarchy->isTerminal(level_iter))
	    conceptsToShow[(*level_iter)->id] = (*level_iter);
	}
	if(kids > 0)
	  nnodes.push_back(kids);
      }
      
      std::map<long int, SpLearningStateConcept>::iterator conceptsToShowIt = conceptsToShow.begin();
      int i, n = conceptsToShow.size();

      bool first = true;
      int level;
      for(i=0; i<n; i++, conceptsToShowIt++) {
	
	mean1 = (*conceptsToShowIt).second->numericalAttributeList[att1]->getMean();
	sigma1 = (*conceptsToShowIt).second->numericalAttributeList[att1]->getSigma();
	mean2 = (*conceptsToShowIt).second->numericalAttributeList[att2]->getMean();
	sigma2 = (*conceptsToShowIt).second->numericalAttributeList[att2]->getSigma();
	level = (*conceptsToShowIt).second->level + 2;
	
	if(first) {
	  first = false;
	  file << "\nplot ";
	} else
	  file << ", \\\n";
	
	file << mean1 << " + " << sigma1 << "*cos(t)," 
	     << mean2 << " + " << sigma2 << "*sin(t) lt " << level << " lw 1.5,\\\n"
	     << mean1 << " + " << "5*cos(t)," 
	     << mean2 << " + " << "5*sin(t) lt " << level << " lw 1.5";
      }
      
      file << "\n\n";

      //Display transitions
      std::map<long int, SpLearningEventConcept>::iterator eventsIt = eventsHierarchy->registeredEvents.begin();
      n = eventsHierarchy->registeredEvents.size();
      double mx1, mx2, my1, my2;
      long int s1, s2;
      SpLearningStateConcept S1, S2;
      for(i=0; i<n; i++, eventsIt++) {
	s1 = (*eventsIt).second->startingState;
	if(conceptsToShow.count(s1) == 0)
	  continue;
	s2 = (*eventsIt).second->arrivingState;
	if(conceptsToShow.count(s2) == 0)
	  continue;
	S1 = eventsHierarchy->registeredStates[s1];
	S2 = eventsHierarchy->registeredStates[s2];
	mx1 = S1->numericalAttributeList[att1]->getMean();
	mx2 = S2->numericalAttributeList[att1]->getMean();
	my1 = S1->numericalAttributeList[att2]->getMean();
	my2 = S2->numericalAttributeList[att2]->getMean();
	file << "set arrow " << i+1 << " from " << mx1 << "," << my1 
	     << ",2 to " << mx2 << "," << my2 << " lw 1.5 lt " << S1->level+2 << " front\n";
	file << std::setprecision(2) << "set label \"" << (*eventsIt).second->P << "\" at " << (mx1+mx2)/2.0 + 15 << "," << (my1+my2)/2.0 << " font \"Helvetica,17\" front\n"; 

      }

      //Display labels
      conceptsToShowIt = conceptsToShow.begin();
      n = conceptsToShow.size();
      
      for(i=0; i<n; i++, conceptsToShowIt++) {
 
	S1 = (*conceptsToShowIt).second;
	mx1 = S1->numericalAttributeList[att1]->getMean();
	my1 = S1->numericalAttributeList[att2]->getMean();
	
	file << "set label \"S" << S1->id << "\" at " << mx1+15 << "," << my1 << " font \"Helvetica,18\" front\n"; 
      }

    }

    file << "\n\n";
    
    int i, n = nnodes.size();
    for(i = 0;i<n;i++)
      file << "\n\n\n# For tree level " << i << ": " << nnodes[i] << "concepts. \n";
    file << std::endl;
    
    file.close();
  }
  
} // end namespace

