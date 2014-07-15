#include "reliabilitymultimodel.h"
#include "MODULES/ModuleInterface.h"
#include "VideoAnalysis.h"
#include <QFile>

ReliabilityMultiModel::ReliabilityMultiModel() {}

ReliabilityMultiModel::ReliabilityMultiModel(Datapool *i_data): m_data(i_data) {} //, P(0.0), R(0.0)

ReliabilityMultiModel::ReliabilityMultiModel(ReliabilityMultiModel *model) {
    m_data = model->m_data;
    multiModelDAG.clear();
    multiModelDAG = model->multiModelDAG;
    binterface = model->binterface;
    //P = model->P;
    //R = model->R;
}

ReliabilityMultiModel::~ReliabilityMultiModel() {}

bool ReliabilityMultiModel::setModelList(QDomElement &parent) {
    if(parent.isNull()) //Node is terminal
        return true;
    QDomNodeList l = parent.elementsByTagName("Model");
    QString name, model_name;
    QDomNode m, o;
    QDomElement e;
    int buffer_size;
    int i=0, n = l.length();
    if(n == 0) //No models
        return true;
    std::set<QString> unique_names;
    std::map<QString, std::set<QString> > dependences;
    std::deque<QString> names_list;

    while(i<n) {
        QDomNode m = l.at(i);
        model_name = XmlCommon::getParameterValue(m, "name");
        if( VideoAnalysis::isValidModelName(model_name) ) {

            buffer_size = XmlCommon::getParameterValue(m, "buffer").toInt();
            if(buffer_size <= 0) {
                AppendToLog("Error reading XML models information. Buffer size undefined or not positive for model '" + name + "'. Default value 5 will be considered.");
                buffer_size  = 5;
            }

            name = XmlCommon::getParameterValue(m, "name");
            if(name == "") {
                name = "DefaultName_" + QString::number(i);
                AppendToLog("Error reading XML models information. Tag name for model '" + model_name + "' is empty. Default value '" + name + "' will be considered.");
            }


            SpReliabilitySingleModelInterface newModel(VideoAnalysis::modelConstructor[model_name]());

            newModel->initFromXML(m_data, buffer_size, name);
            newModel->initInstanceAttributes();
            newModel->initDynamicsAttributes();

            o = XmlCommon::getParameterNode("Parameters", m);
            newModel->setParameters(o);

            multiModelDAG.push_back(newModel);

            //Check unicity of tag names
            if(unique_names.empty())
                unique_names.insert(newModel->name);
            else if(unique_names.count(newModel->name) != 0) { //Change name
                QString proposed;
                int j = 0;
                do {
                    proposed = newModel->name + "_" + QString::number(j++);
                } while(unique_names.count(proposed) != 0);

                AppendToLog("Error reading XML models information. Model of class '" + model_name + "' have repeated name '"
                            + newModel->name + "'. You can use the same model, but named differently. Setting modified name for model: " + proposed);

                newModel->name = proposed;
            }

            names_list.push_back(newModel->name);

            //Check dependences
            o = XmlCommon::getParameterNode("Dependences", m);
            QDomNodeList ll = o.toElement().elementsByTagName("Dependence");
            QString dep_name;
            int k = 0, nn = ll.length();
            QDomNode mm;

            if(nn>0) {
                std::set<QString> deps;
                for(;k<nn; k++) {
                    mm = l.at(k);
                    dep_name = XmlCommon::getParameterValue(mm, "name");
                    deps.insert(dep_name);
                }
                dependences[newModel->name] = deps;
            }

            i++;
        } else{
            i++;
            AppendToLog("Error reading XML models information. XML name '" + model_name + "'' is not a valid model name. Model information will be ignored.");
        }
    }

    if(checkDependences(dependences, names_list)) {
        setDependences(dependences, names_list);
        setIndependentLists();
    } else {
        AppendToLog("Error reading XML models information. Multi-model is not an acyclic DAG.");
        return false;
    }
    return true;
}

//Checks model dependences and applies a topological order of models,
//to allow processing accomplishing dependences. If ordering is
//not achieved, this means that the Directed Graph is not acyclic (error)
bool ReliabilityMultiModel::checkDependences(std::map<QString, std::set<QString> > &dependences, std::deque<QString> &name_list) {

    //If no dependences, every node at first level
    if(dependences.empty())
        return true;

    std::deque<QString> ordered_list;
    std::map<QString, std::set<QString> > clean_list;

    //First check existance of names in list
    std::map<QString, std::set<QString> >::iterator it, it_end = dependences.end();
    std::set<QString>::iterator it2, it2_end;
    std::deque<QString>::iterator it3, it3_end = name_list.end();

    //First clean erroneous names
    for(it = dependences.begin(); it != it_end; it++) {
        std::set<QString> &deps = (*it).second;
        std::set<QString> new_set;
        bool done;
        it2_end = deps.end();
        for(it2 = deps.begin(); it2 != it2_end; it2++) {
            done = false;
            for(it3 = name_list.begin(); it3 != it3_end; it3++) {
                if((*it2) == (*it3)) {
                    new_set.insert(*it2);
                    done = true;
                    break;
                }
            }
            if(!done)
                AppendToLog("Error reading XML models information. XML dependance name '" + (*it2) + "'' is not a valid tag name. Dependance will be ignored.");
        }
        if(!new_set.empty())
            clean_list[(*it).first] = new_set;
    }

    dependences = clean_list;
    if(dependences.empty())
        return true;

    //SET TOPOLOGICAL ORDER BY PROGRESSIVELY ADDING MODELS WITHOUT DEPENDENCES. WHEN REMOVED,
    //THE DEPENDENCES OF THIS MODEL IN OTHERS IS ALSO REMOVED. IF FINALLY NO MODEL REMAINS
    //THIS MEANS THAT THE DIRECTED GRAPH IS ACYCLIC.

    //First insert models without dependences, and erase them from check list
    for(it3 = name_list.begin(); it3 != it3_end; ) {
        if(clean_list.find(*it3) == clean_list.end()) { //Not in list == no dependences
            ordered_list.push_back(*it3);
            it3 = name_list.erase(it3);
        } else
            it3++;
    }

    if(ordered_list.empty()) { //No independent model constitutes an error
        AppendToLog("Error reading XML models information. No independant model.");
        return false;
    }

    //Clean initial dependences
    it_end = clean_list.end();
    it3_end = ordered_list.end();

    for(it3 = ordered_list.begin(); it3 != it3_end; it3++) {
        for(it = clean_list.begin(); it != it_end; it++) {
            std::set<QString> &cur_set = (*it).second;
            if(!cur_set.empty())
                cur_set.erase(*it3);
        }
    }

    //Progressive removal of models, until no more models remaining or no model without
    //dependences is present (no removed model in a loop).
    bool removed_something;

    do {
        removed_something = false;
        it_end = clean_list.end();
        std::deque<QString> toRemove;
        std::deque<QString>::iterator rit, rit_end;
        for(it = clean_list.begin(); it != it_end; it++) {
            if( (*it).second.empty()) {
                toRemove.push_back((*it).first);
                removed_something = true;
            }
        }

        if(removed_something) { //Effectively remove from lists
            //First remove empty ones
            rit_end = toRemove.end();
            for(rit = toRemove.begin(); rit != rit_end; rit++) {
                ordered_list.push_back(*rit);
                clean_list.erase(*rit);
            }
            //Then, remove dependences to erased ones
            if(!clean_list.empty()) {
                it_end = clean_list.end();

                for(rit = toRemove.begin(); rit != rit_end; rit++) {
                    for(it = clean_list.begin(); it != it_end; it++) {
                        std::set<QString> &cur_set = (*it).second;
                        if(!cur_set.empty())
                            cur_set.erase(*rit);
                    }
                }
            }
        }

    } while(removed_something && !name_list.empty());

    if(!name_list.empty()) {
        AppendToLog("Error reading XML models information. XML dependances form a loop. Between models: ");
        it3_end = name_list.end();
        for(it3 = name_list.begin(); it3 != it3_end; it3++)
            AppendToLog("\t" + (*it3));

    }

    name_list = ordered_list;

    if(ordered_list.empty())
        return false;

    return true;
}

void ReliabilityMultiModel::setIndependentLists() {
    std::deque<SpReliabilitySingleModelInterface>::iterator it, it_end = multiModelDAG.end();
    for(it = multiModelDAG.begin(); it != it_end; it++) {
        if((*it)->dependences.empty()) //If forward independent
            forwardIndependentModels.insert((*it)->name);
        if((*it)->dependants.empty()) //If backward independent
            backwardIndependentModels.insert((*it)->name);
    }
}



//Sets model dependences
void ReliabilityMultiModel::setDependences(std::map<QString, std::set<QString> > &dependences, std::deque<QString> &ordered_list) {
    std::deque<SpReliabilitySingleModelInterface> rebuiltDAG;

    std::map<QString, std::set<QString> >::iterator dep_it, dep_it_end = dependences.end();
    std::deque<QString>::iterator it, it_end = ordered_list.end();
    std::deque<SpReliabilitySingleModelInterface>::iterator it3, it2, it_end2 = multiModelDAG.end();
    QString dep_name;

    for(it = ordered_list.begin(); it != it_end; it++) {
        for(it2 = multiModelDAG.begin(); it2 != it_end2; it2++) {
            if((*it) == (*it2)->name) { //If found, add it to new DAG
                SpReliabilitySingleModelInterface current = *it2;

                dep_it = dependences.find((*it2)->name);
                if(dep_it != dep_it_end) { //If the model depends on other models
                    std::set<QString> &dep_set = (*dep_it).second;

                    if(!dep_set.empty()) { //If the model depends on other models (second condition)
                        std::set<QString>::iterator set_it, set_end = dep_set.end();
                        for(set_it = dep_set.begin(); set_it != set_end; set_it++) {
                            dep_name = *set_it;
                            for(it3 = multiModelDAG.begin(); it3 != it_end2; it3++) { //Find dependence model
                                if(dep_name == (*it3)->name) { //If found, add dependence and dependent
                                    SpReliabilitySingleModelInterface dependence = (*it3);
                                    current->dependences.push_back(dependence);
                                    dependence->dependants.push_back(current);
                                    break;
                                }
                            }
                        }
                    }
                }

                rebuiltDAG.push_back(current);
                break;
            }
        }

        if(it2 == it_end2) { //Error: should not happen
            AppendToLog("Error processing DAG. Model not found in multiModelDAG (should not happen). ");
            return;
        }

        multiModelDAG.clear();
        multiModelDAG = rebuiltDAG;

    }


}


bool ReliabilityMultiModel::setParameters(QDomDocument &xmlConfig) {

    QDomElement rootConfig = xmlConfig.documentElement();
    if( rootConfig.tagName() != "MULTI_MODEL" ) {
        AppendToLog("Error reading XML models information. XML content in the configuration file does not correspond to a multi model parameters description. Execution will be aborted.");
        return false;
    }

    if(!setModelList(rootConfig))
        return false;

    if(multiModelDAG.empty()) {
        AppendToLog("Error reading XML models information. No valid model definition found.");
        return false;
    }

    return true;

}


SpReliabilitySingleModelInterface ReliabilityMultiModel::getSingleModelByName(QString name) {
    return getSingleModelByName(name, multiModelDAG);
}

SpReliabilitySingleModelInterface ReliabilityMultiModel::getSingleModelByName(QString name, std::deque<SpReliabilitySingleModelInterface> &models) {

    if(models.empty())
        return SpReliabilitySingleModelInterface();

    std::deque<SpReliabilitySingleModelInterface>::iterator iter = models.begin(), it_end = models.end();

    SpReliabilitySingleModelInterface ret_value;

    for(; iter != it_end; iter++) {
        ret_value = *iter;
        if(ret_value->name == name)
            return ret_value;
    }

    return SpReliabilitySingleModelInterface();
}


bool ReliabilityMultiModel::findSingleModelByName(QString name) {
    return findSingleModelByName(name, multiModelDAG);
}

bool ReliabilityMultiModel::findSingleModelByName(QString name, std::deque<SpReliabilitySingleModelInterface> &models) {
    if(models.empty())
        return false;

    std::deque<SpReliabilitySingleModelInterface>::iterator iter = models.begin(), it_end = models.end();

    for(; iter != it_end; iter++)
        if((*iter)->name == name)
            return true;

    return false;
}


//updates every activated model, in a forward sense.
void ReliabilityMultiModel::update(Blob *blob) {
    if(multiModelDAG.empty())
        return;

    std::deque<SpReliabilitySingleModelInterface>::iterator iter = multiModelDAG.begin(), it_end = multiModelDAG.end();
    bool inputsOK;

    for(; iter != it_end; iter++) {
        SpReliabilitySingleModelInterface m = *iter;

        inputsOK = true;

        if(!m->dependences.empty()) {
            std::deque<SpReliabilitySingleModelInterface>::iterator iter2 = m->dependences.begin(), it2_end = m->dependences.end();
            for(; iter2 != it2_end; iter2++) {
                if((*iter2)->active == false) {
                    inputsOK = false;
                    break;
                }
            }
        }

        if(inputsOK) {
            m->activate(blob);
            if(m->active) {
                m->updateInstance(blob);      //Inserts new information at instance level
                m->updateDynamics();          //Then updates dynamics model
                m->setForwardBlobInterface(); //Sets blob interface resulting from the forward process
            }
        }
    }
}

//updates every activated model, in a backward sense (feedback).
void ReliabilityMultiModel::feedback() {
    if(multiModelDAG.empty())
        return;

    std::deque<SpReliabilitySingleModelInterface>::reverse_iterator iter = multiModelDAG.rbegin(), it_end = multiModelDAG.rend();

    for(; iter != it_end; iter++) {
        SpReliabilitySingleModelInterface m = *iter;

        if(m->active) {
            m->feedback();                 //Uses dependants information for enriching attributes/dynamics information
            m->setProbabilityAndReliability(); //Sets global model reliability and probability
            m->setBackwardBlobInterface(); //Sets blob interface resulting from the whole process
        }
    }
}

void ReliabilityMultiModel::set2DTrajectoryPoint(double &x, double &y, double &R) {

    if(multiModelDAG.empty()) {
        x = y = R = 0.0;
        return;
    }

    std::map<QString, point2D<double> > point;
    std::map<QString, double > Rpoint;
    std::deque<QString> indep;
    std::deque<SpReliabilitySingleModelInterface>::reverse_iterator it, it2, it_end = multiModelDAG.rend(), it_end2;
    double cx, cy, cR;

    //First get independent estimations
    for(it=multiModelDAG.rbegin(); it !=it_end; it++) {
        SpReliabilitySingleModelInterface m = *it;
        if(forwardIndependentModels.count(m->name) > 0)
            indep.push_back(m->name);

        m->setTrajectory2D(cx, cy, cR);          //Sets the estimation of trajectory 2D point for model
        point[m->name] = point2D<double>(cx, cy);
        Rpoint[m->name] = R;
    }

    if(point.size() == 1) {
        x = point.begin()->second.x;
        y = point.begin()->second.y;
        R = Rpoint.begin()->second;
        return;
    }

    std::map<QString, point2D<double> > composite_point;
    std::map<QString, double > composite_Rpoint;
    int j;
    double r, xsum, ysum, Rsum;
    point2D<double> p;

    //Then, set level estimations:
    for(it=multiModelDAG.rbegin(); it !=it_end; it++) {
        SpReliabilitySingleModelInterface m = *it;
        if(m->dependants.size() == 0) {
            composite_point[m->name] = point[m->name];
            composite_Rpoint[m->name] = Rpoint[m->name];
        } else {
            it_end2 = m->dependants.rend();
            xsum = 0.0; ysum = 0.0, Rsum = 0.0; j = 0;
            for(it2=m->dependants.rbegin(); it2 !=it_end2; it2++) {
                p = composite_point[(*it2)->name];
                r = composite_Rpoint[(*it2)->name];
                xsum += r * p.x;
                ysum += r * p.y;
                Rsum += r;
                j++;
            }
            p.x = xsum/Rsum;
            p.y = ysum/Rsum;

            if(Rsum/j > Rpoint[m->name]) { //Max reliability criteria
                composite_point[m->name] = p;
                composite_Rpoint[m->name] = Rsum/j;
            } else {
                composite_point[m->name] = point[m->name];
                composite_Rpoint[m->name] = Rpoint[m->name];
            }
        }
    }

    //Last phase, mean position among independent models:
    std::deque<QString>::iterator nit, nend = indep.end();
    xsum = 0.0; ysum = 0.0, Rsum = 0.0; j = 0;
    for(nit=indep.begin(); nit != nend; nit++) {
        p = composite_point[(*nit)];
        r = composite_Rpoint[(*nit)];
        xsum += r * p.x;
        ysum += r * p.y;
        Rsum += r;
        j++;
    }

    x = xsum/Rsum;
    y = ysum/Rsum;
    R = Rsum/j;

}


// Generate the ROI estimation using binterface_backward (obtained after backward/feedback process) of independent models:

Rectangle<int> ReliabilityMultiModel::getVisualEvidenceEstimator(double &R) {
    std::deque<SpReliabilitySingleModelInterface>::iterator it, it_end = multiModelDAG.end();
    Rectangle<int> r, rmerge;
    int j = 0;
    double Rmod, Rsum = 0, lsum = 0, rsum = 0, tsum = 0, bsum = 0;

    for(it=multiModelDAG.begin(); it !=it_end; it++) {
        if(forwardIndependentModels.count((*it)->name) > 0) {
            r = (*it)->getEstimator(Rmod);
            lsum += r.xleft  *Rmod;
            rsum += r.xright *Rmod;
            tsum += r.ytop   *Rmod;
            bsum += r.ybottom*Rmod;
            Rsum += Rmod;
            j++;
        }
    }
    //By dimension: mean between dependants and model dimension,
    //weighted by their reliability
    rmerge.xleft   = lsum/Rsum;
    rmerge.xright  = rsum/Rsum;
    rmerge.ytop    = tsum/Rsum;
    rmerge.ybottom = bsum/Rsum;
    rmerge.width   = rmerge.xright - rmerge.xleft + 1;
    rmerge.height  = rmerge.ybottom - rmerge.ytop + 1;

    //Mean between independant models reliability
    R = Rsum/j;
    return rmerge;
}

void ReliabilityMultiModel::getGlobalProbability(double &R, double &P) {
    if(multiModelDAG.empty()) {
        R = P = 0;
        return;
    }

    std::deque<SpReliabilitySingleModelInterface>::iterator it, it_end = multiModelDAG.end();
    int j = 0;
    double cr, cp, sumr = 0.0, sump = 0.0;

    for(it=multiModelDAG.begin(); it !=it_end; it++) {
        if(forwardIndependentModels.count((*it)->name) > 0) {
            cr = (*it)->R_backward;
            cp = (*it)->P;
            sump += cp*cr;
            sumr += cr;
            j++;
        }
    }

    R = sumr/j;
    P = (R == 0.0) ? 0.0 : sump/sumr;
}


//Rendering Functions
std::ostream& operator<<(std::ostream& out, const ReliabilityMultiModel& rmm) {
//ACAAAA!!!: Implementar
    return out;
}





