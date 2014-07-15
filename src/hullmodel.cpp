#include "hullmodel.h"

HullModel::HullModel() {
    off_x = off_y = 0;
}

HullModel::HullModel(HullModel &h) {
    local_hull = h.local_hull;
    hull = h.hull;
    off_x = h.off_x;
    off_y = h.off_y;
    axis1 = h.axis1;
    axis2 = h.axis2;
    id = h.id;
    segs = h.segs;
}

HullModel::HullModel(SpHullModel h) {
    local_hull = h->local_hull;
    hull = h->hull;
    off_x = h->off_x;
    off_y = h->off_y;
    axis1 = h->axis1;
    axis2 = h->axis2;
    id = h->id;
    segs = h->segs;
}


HullModel::HullModel(HullModel &h, int off_x, int off_y) {
    local_hull = h.local_hull;
    hull = h.local_hull;
    setOffset(off_x, off_y);
}

void HullModel::setOffset(int i_off_x, int i_off_y) {
    off_x = i_off_x;
    off_y = i_off_y;
    if(off_x != 0 || off_y != 0) {
        int i, n = hull.size();
        for(i=0; i<n; i++) {
            hull[i].x += off_x;
            hull[i].y += off_y;
        }
    }
}

void HullModel::setDistanceVector(SceneModel *sm) {
    int i, n = segs.size(), dx, dy;
    double x1, y1, x2, y2;

    dists.resize(n);
    hull_dists.resize(n);

    for(i=0; i<n; i++) {
        segment2D<float> &s = segs[i];
        segment2D<float> &hs = hull_segs[i];
        sm->imgToHomographyCoords(sm->h_matrix, s.first.x, s.first.y, &x1, &y1);
        sm->imgToHomographyCoords(sm->h_matrix, s.last.x,  s.last.y,  &x2, &y2);
        dx = x2 - x1;
        dy = y2 - y1;
        dists[i] = sqrt(dx*dx + dy*dy);
        sm->imgToHomographyCoords(sm->h_matrix, hs.first.x, hs.first.y, &x1, &y1);
        sm->imgToHomographyCoords(sm->h_matrix, hs.last.x,  hs.last.y,  &x2, &y2);
        dx = x2 - x1;
        dy = y2 - y1;
        hull_dists[i] = sqrt(dx*dx + dy*dy);
    }

}

HullModel::~HullModel() {}
