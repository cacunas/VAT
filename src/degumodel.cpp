#include "degumodel.h"
#include "Datapool.h"

/*
3-6 meses
Largo con cola : 27+-1 cm
Largo sin cola: 18,5 +-2 cm
Ancho: 11+-1 cm

Mayores a 1 año
Largo con cola : 30,5+-1 cm
Largo sin cola: 21,6 +-3 cm
Ancho: 13,6+-3 cm
*/

//Taking mean length for checking if tail can be present
double DeguModel::DEGU_SIZE_NO_TAIL   = 20.0;
double DeguModel::DEGU_SIZE_WITH_TAIL = 28.5;
//double DeguModel::DEGU_SIZE_NO_TAIL   = 30.0;
//double DeguModel::DEGU_SIZE_WITH_TAIL = 40.0;


DeguModel::DeguModel(int i_size): size(i_size) {
    numFramesSeen = 0;
    insertionType = 0;
}

DeguModel::~DeguModel() {}

void DeguModel::insert(Datapool *i_data, SpHullModel h, SpRMMMobileObject o) {

    SpDeguInstance newDegu(new DeguInstance());

    newDegu->hull = h;
    newDegu->centerPosition.x = RECT_XCENTER(&o->multiModel.binterface.bbox);
    newDegu->centerPosition.y = RECT_YCENTER(&o->multiModel.binterface.bbox);


    if(instances.size() == size)
        instances.pop_front();
    instances.push_back(newDegu);

    double x1, y1, x2, y2;
    cv::Point p1 = h->axis1, p2 = h->axis2;
    SceneModel *s = i_data->sceneModel;

    s->imgToHomographyCoords(s->h_matrix, p1.x, p1.y, &x1, &y1);
    s->imgToHomographyCoords(s->h_matrix, p2.x, p2.y, &x2, &y2);
    double dx = x2 - x1, dy = y2 - y1;
    double d = sqrt(dx*dx + dy*dy);
    int n = h->segs.size(), n_2 = n/2;

    std::cout << "Degu size: " << d << "[cm]" << std::endl;

    newDegu->length = d;

    if (n_2 < 2)
        return;

    std::vector<double> &dists = h->dists;

    if(dists.size() != n) {
        if(!dists.empty()) dists.clear();
        h->setDistanceVector(s);
    }

    int i, j;
    double acum1 = 0.0, acum2 = 0.0;
    for(i=0, j=n-1; i<n_2; i++, j--) {
        acum1 += dists[i];
        acum2 += dists[j];
    }

    newDegu->headByHull = true;
    newDegu->headByVelocity = false;

    if(d > DeguModel::DEGU_SIZE_NO_TAIL
        ) { //TAIL
            insertionType = 3;
            newDegu->headPosition = acum1 > acum2 ? point2D<int>(p1.x, p1.y) : point2D<int>(p2.x, p2.y);
            newDegu->headAtPoint1 = acum1 > acum2 ? true : false;
            newDegu->Rhull = 1.0 - fabs(d - DeguModel::DEGU_SIZE_WITH_TAIL)/DeguModel::DEGU_SIZE_WITH_TAIL;
            if(numFramesSeen > 0) { //At least axis should be fine, check coherence
                SpDeguInstance in = instances.back();
                if(in->Rhull > 0.9) {
                    point2D<int> lastCenter = trajectory.back();
                    dx = newDegu->centerPosition.x - lastCenter.x,
                    dy = newDegu->centerPosition.y - lastCenter.y;
                    int dx1 = in->headPosition.x + dx - p1.x,
                        dy1 = in->headPosition.y + dy - p1.y,
                        dx2 = in->headPosition.x + dx - p2.x,
                        dy2 = in->headPosition.y + dy - p2.y;
                    if(dx1*dx1+dy1*dy1 > dx2*dx2+dy2*dy2) { //More near to point 2
                        if(newDegu->headAtPoint1 == true) { //No coincidence
                            insertionType = 4;
                            newDegu->Rhull = in->Rhull*0.9;
                            newDegu->headPosition = point2D<int>(p2.x, p2.y);
                        }
                    } else { //More near to point 1
                        if(newDegu->headAtPoint1 == false) { //No coincidence
                            insertionType = 4;
                            newDegu->Rhull = in->Rhull*0.9;
                            newDegu->headPosition = point2D<int>(p1.x, p1.y);
                        }
                    }
                }
            }
        }

    //No tail
    //if(   d <= DeguModel::DEGU_SIZE_NO_TAIL/2.0 ) {
/*    //   || fabs(DeguModel::DEGU_SIZE_WITH_TAIL - d) > fabs(DeguModel::DEGU_SIZE_NO_TAIL - d)) {
        //Descending with distance to DEGU_SIZE_WITH_TAIL
        newDegu->Rhull = 1.0 - fabs(d - DeguModel::DEGU_SIZE_WITH_TAIL)/DeguModel::DEGU_SIZE_WITH_TAIL;
        if(d < DeguModel::DEGU_SIZE_NO_TAIL / 2) { //Too little, use last distance to position
            if(numFramesSeen > 0) { //Keep distance to center
                insertionType = 0;
                point2D<int> lastCenter = trajectory.back(),
                             lastHead   = head_trajectory.back();
                dx = lastHead.x - lastCenter.x,
                dy = lastHead.y - lastCenter.y;
                newDegu->headPosition = point2D<int>(newDegu->centerPosition.x+dx,
                                                     newDegu->centerPosition.y+dy);
            } else {//Is the first one, take center of axis
                insertionType = 1;
                newDegu->headPosition = point2D<int>((p1.x+p2.x)/2, (p1.y+p2.y)/2);
            }
        } else {
            insertionType = 2;
            newDegu->headPosition = acum1 < acum2 ? point2D<int>(p1.x, p1.y) : point2D<int>(p2.x, p2.y);
            newDegu->headAtPoint1 = acum1 < acum2 ? true : false;
            if(numFramesSeen > 0) { //At least axis should be fine, check coherence
                SpDeguInstance in = instances.back();
                if(in->Rhull > newDegu->Rhull*2) {
                    point2D<int> lastCenter = trajectory.back();
                    dx = newDegu->centerPosition.x - lastCenter.x,
                    dy = newDegu->centerPosition.y - lastCenter.y;
                    int dx1 = in->headPosition.x + dx - p1.x,
                        dy1 = in->headPosition.y + dy - p1.y,
                        dx2 = in->headPosition.x + dx - p2.x,
                        dy2 = in->headPosition.y + dy - p2.y;
                    if(dx1*dx1+dy1*dy1 > dx2*dx2+dy2*dy2) { //More near to point 2
                        if(newDegu->headAtPoint1 == true) { //No coincidence
                            insertionType = 4;
                            newDegu->Rhull = in->Rhull*0.9;
                            newDegu->headPosition = point2D<int>(p2.x, p2.y);
                        }
                    } else { //More near to point 1
                        if(newDegu->headAtPoint1 == false) { //No coincidence
                            insertionType = 4;
                            newDegu->Rhull = in->Rhull*0.9;
                            newDegu->headPosition = point2D<int>(p1.x, p1.y);
                        }
                    }
                }
            }
        }
    } else { //TAIL
        insertionType = 3;
        newDegu->headPosition = acum1 > acum2 ? point2D<int>(p1.x, p1.y) : point2D<int>(p2.x, p2.y);
        newDegu->headAtPoint1 = acum1 > acum2 ? true : false;
        newDegu->Rhull = 1.0 - fabs(d - DeguModel::DEGU_SIZE_WITH_TAIL)/DeguModel::DEGU_SIZE_WITH_TAIL;
        if(numFramesSeen > 0) { //At least axis should be fine, check coherence
            SpDeguInstance in = instances.back();
            if(in->Rhull > 0.9) {
                point2D<int> lastCenter = trajectory.back();
                dx = newDegu->centerPosition.x - lastCenter.x,
                dy = newDegu->centerPosition.y - lastCenter.y;
                int dx1 = in->headPosition.x + dx - p1.x,
                    dy1 = in->headPosition.y + dy - p1.y,
                    dx2 = in->headPosition.x + dx - p2.x,
                    dy2 = in->headPosition.y + dy - p2.y;
                if(dx1*dx1+dy1*dy1 > dx2*dx2+dy2*dy2) { //More near to point 2
                    if(newDegu->headAtPoint1 == true) { //No coincidence
                        insertionType = 4;
                        newDegu->Rhull = in->Rhull*0.9;
                        newDegu->headPosition = point2D<int>(p2.x, p2.y);
                    }
                } else { //More near to point 1
                    if(newDegu->headAtPoint1 == false) { //No coincidence
                        insertionType = 4;
                        newDegu->Rhull = in->Rhull*0.9;
                        newDegu->headPosition = point2D<int>(p1.x, p1.y);
                    }
                }
            }
        }
    }
*/

    headPosition = newDegu->headPosition;
    trajectory.push_back(newDegu->centerPosition);
    head_trajectory.push_back(newDegu->headPosition);
    frame_numbers.push_back(i_data->frameNumber);
    newDegu->frameNumber = i_data->frameNumber;
    numFramesSeen++;
}

void DeguModel::move(Datapool *i_data, SpRMMMobileObject o) {
    SpDeguInstance newDegu(new DeguInstance());
    insertionType = 4;
    int dx, dy;

    newDegu->centerPosition.x = RECT_XCENTER(&o->multiModel.binterface.bbox);
    newDegu->centerPosition.y = RECT_YCENTER(&o->multiModel.binterface.bbox);

    if(instances.size() > 0 ) {
        SpDeguInstance in = instances.back();
        if(in->hull.isNull()) {
            SpHullModel nl;
            newDegu->hull = nl;
            newDegu->length = 0;
            if(numFramesSeen > 0) { //Keep distance to center
                point2D<int> lastCenter = trajectory.back(),
                             lastHead   = head_trajectory.back();
                dx = lastHead.x - lastCenter.x,
                dy = lastHead.y - lastCenter.y;
                newDegu->headPosition = point2D<int>(newDegu->centerPosition.x+dx,
                                                     newDegu->centerPosition.y+dy);
            } else {//Is the first one, take center position
                newDegu->headPosition = point2D<int>(newDegu->centerPosition.x,
                                                     newDegu->centerPosition.y);
                newDegu->Rhull = 0.0;
            }
        } else { //Move last hull
            SpHullModel new_hull(new HullModel(in->hull));
            cv::Point p1 = new_hull->axis1, p2 = new_hull->axis2;
            dx = newDegu->centerPosition.x - in->centerPosition.x;
            dy = newDegu->centerPosition.y - in->centerPosition.y;
            new_hull->off_x += dx;
            new_hull->off_y += dy;
            newDegu->hull = new_hull;

            if(numFramesSeen > 0) { //Keep distance to center
                point2D<int> lastCenter = trajectory.back(),
                             lastHead   = head_trajectory.back();
                dx = lastHead.x - lastCenter.x,
                dy = lastHead.y - lastCenter.y;
                newDegu->headPosition = point2D<int>(newDegu->centerPosition.x+dx,
                                                     newDegu->centerPosition.y+dy);
            } else {//Is the first one, take center of axis
                newDegu->headPosition = point2D<int>((p1.x+p2.x)/2, (p1.y+p2.y)/2);
                newDegu->Rhull = 0.0;
            }
        }
    } else {
        SpHullModel nl;
        newDegu->hull = nl;
        newDegu->length = 0;
        newDegu->headPosition = point2D<int>(newDegu->centerPosition.x,
                                             newDegu->centerPosition.y);
        newDegu->Rhull = 0.0;
    }

    newDegu->headByHull = true;
    newDegu->headByVelocity = false;


    headPosition = newDegu->headPosition;
    trajectory.push_back(newDegu->centerPosition);
    head_trajectory.push_back(newDegu->headPosition);
    frame_numbers.push_back(i_data->frameNumber);
    newDegu->frameNumber = i_data->frameNumber;
    numFramesSeen++;

    if(instances.size() == size)
        instances.pop_front();
    instances.push_back(newDegu);
}

//Keep model coherent
void DeguModel::keep(int frame) {
    SpDeguInstance newDegu(new DeguInstance());
    int dx, dy;
    insertionType = 5;

    if(instances.size() > 0 ) {
        SpDeguInstance in = instances.back();
        if(instances.size() > 1 ) { //Can get velocity
            SpDeguInstance in2 = instances.back();
            int dxcenter = in->centerPosition.x - in2->centerPosition.x,
                dycenter = in->centerPosition.y - in2->centerPosition.y,
                dframe    = in->frameNumber - in2->frameNumber,
                dnewframe = frame - in->frameNumber;

            newDegu->centerPosition.x = (dxcenter*dnewframe)/dframe;
            newDegu->centerPosition.y = (dycenter*dnewframe)/dframe;
        } else {
            newDegu->centerPosition.x = in->centerPosition.x;
            newDegu->centerPosition.y = in->centerPosition.y;
        }

        if(in->hull.isNull()) {
            SpHullModel nl;
            newDegu->hull = nl;
            newDegu->length = 0;
            if(numFramesSeen > 0) { //Keep distance to center
                point2D<int> lastCenter = trajectory.back(),
                             lastHead   = head_trajectory.back();
                dx = lastHead.x - lastCenter.x,
                dy = lastHead.y - lastCenter.y;
                newDegu->headPosition = point2D<int>(newDegu->centerPosition.x+dx,
                                                     newDegu->centerPosition.y+dy);
            } else {//Is the first one, take center of axis
                newDegu->headPosition = point2D<int>(newDegu->centerPosition.x,
                                                     newDegu->centerPosition.y);
                newDegu->Rhull = 0.0;
            }
        } else { //Move last hull
            SpHullModel new_hull(new HullModel(in->hull));
            cv::Point p1 = new_hull->axis1, p2 = new_hull->axis2;
            dx = newDegu->centerPosition.x - in->centerPosition.x;
            dy = newDegu->centerPosition.y - in->centerPosition.y;
            new_hull->off_x += dx;
            new_hull->off_y += dy;
            newDegu->hull = new_hull;
            if(numFramesSeen > 0) { //Keep distance to center
                point2D<int> lastCenter = trajectory.back(),
                             lastHead   = head_trajectory.back();
                dx = lastHead.x - lastCenter.x,
                dy = lastHead.y - lastCenter.y;
                newDegu->headPosition = point2D<int>(newDegu->centerPosition.x+dx,
                                                     newDegu->centerPosition.y+dy);
            } else {//Is the first one, take center of axis
                newDegu->headPosition = point2D<int>((p1.x+p2.x)/2, (p1.y+p2.y)/2);
                newDegu->Rhull = 0.0;
            }

        }
    }

    newDegu->headByHull = true;
    newDegu->headByVelocity = false;

    headPosition = newDegu->headPosition;
    trajectory.push_back(newDegu->centerPosition);
    head_trajectory.push_back(newDegu->headPosition);
    frame_numbers.push_back(frame);
    newDegu->frameNumber = frame;
    numFramesSeen++;

    if(instances.size() == size)
        instances.pop_front();
    instances.push_back(newDegu);

}
