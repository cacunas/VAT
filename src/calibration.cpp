#include "calibration.h"
#include "object.h"
#include "VideoAnalysis.h"
#include <QFile>
#include <cstdlib>

//double SceneModel::m_OneMeterRepresentation = 100; //defaults to centimeters

SceneModel::SceneModel() {
    xmlScene = NULL;
    QSharedPointer<point3D<double> > cfocal(new point3D<double>());
    camera_focal_point = cfocal;
    QSharedPointer<point2D<double> > cfocal2D(new point2D<double>());
    camera_focal_point2D = cfocal2D;
    pmatrix_filled = hmatrix_filled = false;
    badFocal = true;
}

SceneModel::~SceneModel() {
    if(xmlScene != NULL)
        delete xmlScene;
}

position SceneModel::setPosition(std::string pos) {
    if(pos == "MiddleCenter")
        return MiddleCenter;
    if(pos == "Left")
        return Left;
    if(pos == "Right")
        return Right;
    if(pos == "Top")
        return Top;
    if(pos == "TopLeft")
        return TopLeft;
    if(pos == "BottomLeft")
        return BottomLeft;
    if(pos == "TopRight")
        return TopRight;
    if(pos == "BottomRight")
        return BottomRight;
    return Bottom;
}

//Get considered position given a blob
void SceneModel::getXY(position pos, double &X, double &Y, Blob b) {
    Rectangle<int> *r = &b.bbox;
    X = r->xleft; //Left
    if(pos & Right)
        X += r->width - 1;
    else if(!(pos & Left)) //Center
        X += r->width / 2;

    Y = r->ytop; //Top
    if(pos & Bottom)
        Y += r->height - 1;
    else if(!(pos & Top)) //Middle
        Y += r->height/2;
}

int SceneModel::worldToImgCoords(perspective_matrix persp_mat,
                        double x, double y, double z, double *X, double *Y) {
    double Xt, Yt, t;

    t = persp_mat[8] * x + persp_mat[9] * y + persp_mat[10] * z + persp_mat[11];
    if (t == 0.0)
        return -1;

    if (X != NULL) {
        Xt = persp_mat[0] * x + persp_mat[1] * y + persp_mat[2] * z + persp_mat[3];
        *X = Xt/t;
    }
    if (Y != NULL) {
        Yt = persp_mat[4] * x + persp_mat[5] * y + persp_mat[6] * z + persp_mat[7];
        *Y = Yt/t;
    }
    return 0;
}

int SceneModel::homographyToImgCoords(homography_matrix H,
                          double X2, double Y2,
                          double *X, double *Y) {
    double
        A = X2*H[6] - H[0],
        B = X2*H[7] - H[1],
        C = X2*H[8] - H[2],
        D = Y2*H[6] - H[3],
        E = Y2*H[7] - H[4],
        F = Y2*H[8] - H[5];
    *Y = (F*A - D*C)/(B*D - E*A);
    *X = -((*Y)*B + C)/A;
    return 0;
}


int SceneModel::imgToHomographyCoords(homography_matrix H,
                          double X, double Y,
                          double *X2, double *Y2) {
    double Xt, Yt, t;

    t = H[6] * X + H[7] * Y + H[8];
    if (t == 0.0)
        return -1;

    if (X2 != NULL) {
        Xt = H[0] * X + H[1] * Y + H[2];
        *X2 = Xt/t;
    }
    if (Y2 != NULL) {
        Yt = H[3] * X + H[4] * Y + H[5];
        *Y2 = Yt/t;
    }
    return 0;

}


int SceneModel::imgToWorldCoordsGivenHeight(perspective_matrix persp_mat,
                                     double X, double Y, double z, double *x, double *y) {
    double p00 = persp_mat[0];
    double p01 = persp_mat[1];
    double p02 = persp_mat[2];
    double p03 = persp_mat[3];
    double p10 = persp_mat[4];
    double p11 = persp_mat[5];
    double p12 = persp_mat[6];
    double p13 = persp_mat[7];
    double p20 = persp_mat[8];
    double p21 = persp_mat[9];
    double p22 = persp_mat[10];
    double p23 = persp_mat[11];

    double den = (p21*p10 - p20*p11) * X + (p01*p20 - p00*p21) * Y + p00*p11 - p01*p10;

    double A1 = X * p20 - p00;
    double A2 = Y * p20 - p10;
    double B1 = X * p21 - p01;
    double B2 = Y * p21 - p11;
    double C1 = p02 * z + p03 - (p22 * z + p23) * X;
    double C2 = p12 * z + p13 - (p22 * z + p23) * Y;

    if(den==0.0)
      return -1;

    if(x!=NULL)
      *x = (B2 * C1 - C2 * B1) / den;

    if(y!=NULL)
      *y = (A1 * C2 - C1 * A2) / den;

    return 0;
}

bool SceneModel::readParameters(QString fileName) {
    pmatrix_filled = hmatrix_filled = false;

    QDomElement root = xmlScene->documentElement();
    if( root.tagName() != "scene" ) {
        AppendToLog("SceneModel: Error reading XML information. XML content in the scene configuration file '" + fileName + "' does not correspond to a scene description. Execution will be aborted.");
        return false;
    }

    QDomNode n = root.firstChild();
    while( !n.isNull() ) {
        QDomElement e = n.toElement();
        if( !e.isNull() ) {
            if( e.tagName() == "perspective_matrix" ) {
                pmatrix_filled = true;
                p_matrix[0]  = e.attribute( "m00", "1" ).toDouble();
                p_matrix[1]  = e.attribute( "m01", "0" ).toDouble();
                p_matrix[2]  = e.attribute( "m02", "0" ).toDouble();
                p_matrix[3]  = e.attribute( "m03", "0" ).toDouble();
                p_matrix[4]  = e.attribute( "m10", "0" ).toDouble();
                p_matrix[5]  = e.attribute( "m11", "1" ).toDouble();
                p_matrix[6]  = e.attribute( "m12", "0" ).toDouble();
                p_matrix[7]  = e.attribute( "m13", "0" ).toDouble();
                p_matrix[8]  = e.attribute( "m20", "0" ).toDouble();
                p_matrix[9]  = e.attribute( "m21", "0" ).toDouble();
                p_matrix[10] = e.attribute( "m22", "1" ).toDouble();
                p_matrix[11] = e.attribute( "m23", "1" ).toDouble();
            } else if( e.tagName() == "homography_matrix" ) {
                hmatrix_filled = true;
                h_matrix[0] = e.attribute( "h00", "1" ).toDouble();
                h_matrix[1] = e.attribute( "h01", "0" ).toDouble();
                h_matrix[2] = e.attribute( "h02", "0" ).toDouble();
                h_matrix[3] = e.attribute( "h10", "0" ).toDouble();
                h_matrix[4] = e.attribute( "h11", "1" ).toDouble();
                h_matrix[5] = e.attribute( "h12", "0" ).toDouble();
                h_matrix[6] = e.attribute( "h20", "0" ).toDouble();
                h_matrix[7] = e.attribute( "h21", "0" ).toDouble();
                h_matrix[8] = e.attribute( "h22", "1" ).toDouble();

                h_trans_i.setMatrix(h_matrix[0], h_matrix[3], h_matrix[6],
                                  h_matrix[1], h_matrix[4], h_matrix[7],
                                  h_matrix[2], h_matrix[5], h_matrix[8]);

                h_trans.setMatrix(1, 0, 0,
                                  0, 1, 0,
                                  0, 0, 1);
                bool invertible;

                if( (invertible = h_trans_i.isInvertible()) ) {
                    h_trans = h_trans_i.inverted(&invertible);
                } else {
                    h_trans = h_trans_i.adjoint();
                }


            } else if( e.tagName() == "ZoneList" ) {
                QDomNode m = n.firstChild();
                while( !m.isNull() ) {
                    QDomElement e = m.toElement();
                    if( !e.isNull() ) {
                        if( e.tagName() == "Zone" ) {
                            QSharedPointer<world::Zone> pZone(new world::Zone());
                            pZone->id = e.attribute("id","-1").toInt();
                            pZone->name = e.attribute("name","None");

                            QDomNode o = m.firstChild();
                            while( !o.isNull() ) {
                                QDomElement f = o.toElement();
                                if( !f.isNull() ) {
                                    if( f.tagName() == "OutlineList" ) {
                                        QSharedPointer< polygon3D<double> > pol(new polygon3D<double>());
                                        pZone->outline3D = pol;

                                        QDomNode p = o.firstChild();
                                        while( !p.isNull() ) {
                                            QDomElement g = p.toElement();
                                            if( !g.isNull() ) {
                                                if( g.tagName() == "Point" ) {
                                                    point3D<double> point;
                                                    point.x = g.attribute("x","0").toDouble();
                                                    point.y = g.attribute("y","0").toDouble();
                                                    point.z = g.attribute("z","0").toDouble();
                                                    pol->points.push_back(point);
                                                }
                                            }
                                            p = p.nextSibling();
                                        }
                                        pZone->outline = pol->projectPolygon2D(p_matrix);
                                    }
                                }
                                o = o.nextSibling();
                            }
                            Zones.push_back(pZone);
                        }
                    }
                    m = m.nextSibling();
                }
            } else if( e.tagName() == "ContextObjectsList" ) {
                QDomNode m = n.firstChild();
                while( !m.isNull() ) {
                    QDomElement e = m.toElement();
                    if( !e.isNull() ) {
                        if( e.tagName() == "ContextObject" ) {
                            QSharedPointer<world::ContextObject> pObject(new world::ContextObject());
                            pObject->id = e.attribute("id","-1").toInt();
                            pObject->name = e.attribute("name","None");
                            pObject->height = e.attribute("height","10").toDouble();
                            pObject->hollow = false;
                            pObject->noRoof = false;
                            QDomNode o = m.firstChild();
                            while( !o.isNull() ) {
                                QDomElement f = o.toElement();
                                if( !f.isNull() ) {
                                    if( f.tagName() == "OutlineList" ) {
                                        QSharedPointer< polygon3D<double> > pol(new polygon3D<double>());
                                        pObject->outline = pol;

                                        QDomNode p = o.firstChild();
                                        while( !p.isNull() ) {
                                            QDomElement g = p.toElement();
                                            if( !g.isNull() ) {
                                                if( g.tagName() == "Point" ) {
                                                    point3D<double> point;
                                                    point.x = g.attribute("x","0").toDouble();
                                                    point.y = g.attribute("y","0").toDouble();
                                                    point.z = g.attribute("z","0").toDouble();
                                                    pol->points.push_back(point);
                                                }
                                            }
                                            p = p.nextSibling();
                                        }
                                        //pObject->outline2DOnImage = pol->projectPolygon2D(p_matrix);
                                        //pObject->poly2DOfObjectBase = pol->projectPolygon2D(p_matrix);
                                        //pObject->poly2DOfObjectRoof = pol->projectPolygon2D(p_matrix);
                                    }
                                }
                                o = o.nextSibling();
                            }
                            contextObjects.push_back(pObject);
                        }
                    }
                    m = m.nextSibling();
                }
            } else if( e.tagName() == "ZoneHomographyList" ) {
                QDomNode m = n.firstChild();
                QString stype;
                while( !m.isNull() ) {
                    QDomElement e = m.toElement();
                    if( !e.isNull() ) {
                        if( e.tagName() == "ZoneHomography" ) {
                            stype = e.attribute("type","-1");
                            QSharedPointer<world::ZoneH> pZone;
                            if(stype == "Ellipse") {
                                QSharedPointer<world::ZoneH> auxZone(new world::ZoneHEllipse());
                                auxZone->type = world::Z2D_ellipse;
                                pZone = auxZone;
                            } else { //defaults to polygon
                                QSharedPointer<world::ZoneH> auxZone(new world::ZoneHPolygon());
                                auxZone->type = world::Z2D_polygon;
                                pZone = auxZone;
                            }

                            pZone->id = e.attribute("id","-1").toInt();
                            pZone->name = e.attribute("name","None");

                            QDomNode o = m.firstChild();
                            pZone->setParameters(o, h_matrix);

                            ZonesH.push_back(pZone);
                        }
                    }
                    m = m.nextSibling();
                }
            } else if( e.tagName() == "Zone2DList" ) {
                QDomNode m = n.firstChild();
                QString stype;
                while( !m.isNull() ) {
                    QDomElement e = m.toElement();
                    if( !e.isNull() ) {
                        if( e.tagName() == "Zone2D" ) {
                            stype = e.attribute("type","-1");
                            QSharedPointer<world::Zone2D> pZone;
                            if(stype == "Ellipse") {
                                QSharedPointer<world::Zone2D> auxZone(new world::Zone2DEllipse());
                                auxZone->type = world::Z2D_ellipse;
                                pZone = auxZone;
                            } else { //defaults to polygon
                                QSharedPointer<world::Zone2D> auxZone(new world::Zone2DPolygon());
                                auxZone->type = world::Z2D_polygon;
                                pZone = auxZone;
                            }

                            pZone->id = e.attribute("id","-1").toInt();
                            pZone->name = e.attribute("name","None");

                            QDomNode o = m.firstChild();
                            pZone->setParameters(o);

                            Zones2D.push_back(pZone);
                        }
                    }
                    m = m.nextSibling();
                }
            }
        }
        n = n.nextSibling();
    }

    if(!pmatrix_filled && !hmatrix_filled) {
        AppendToLog("SceneModel: Error defining XML scene information. XML scene description does not contain a perspective_matrix or a homography_matrix description, in file '" + fileName + "'. Execution will be aborted.");
        return false;
    }

    return true;
}

bool SceneModel::readScene(QString fileName) {
    if(xmlScene != NULL)
        delete xmlScene;
    xmlScene = new QDomDocument( "scene" );
    QFile file( fileName );
    if( !file.open(QIODevice::ReadOnly) ) {
        AppendToLog("SceneModel: Error opening file. The scene configuration file '" + fileName + "' could not be opened. Execution will be aborted.");
        return false;
    }


    QString error;
    int line, column;
    if( !xmlScene->setContent( &file, &error, &line, &column) ) {
        AppendToLog("SceneModel: Error reading XML file in line "
                    + QString::number(line) + ", column "
                    + QString::number(column) + ":" + error
                    + ". XML content in the scene configuration file '"
                    + fileName + "' could not be extracted. Execution will be aborted.");
        file.close();
        return false;
    }
    file.close();

    return readParameters(fileName);
}

//Function which returns in arguments camx, camy, camz the point obtained intersecting the vision lines
//generated from the perspective matrix for two arbitrary image points.
//It then computes the 3d point where the focal point is.
bool SceneModel::computeFocalPoint(perspective_matrix pp, int img_width, int img_height, double *camx, double *camy, double *camz) {

    double xA[2], yA[2], zA[2], xB[2], yB[2], zB[2], kxA, kyA, kxB, kyB;
    //Get 3D line equations A and B for two arbitrary 2D points.
    // x = x1 + (z - z1) * (x2 - x1) / (z2 - z1) = x1 + (z - z1) * kx;
    // y = y1 + (z - z1) * (y2 - y1) / (z2 - z1) = y1 + (z - z1) * ky;
    zA[0] = zB[0] = 0.0;
    zA[1] = zB[1] = m_OneMeterRepresentation;

    //Equation for line A
    imgToWorldCoordsGivenHeight(pp,(double)img_width * 3.0 / 4.0, (double)img_height / 4.0, zA[0], &xA[0], &yA[0]);
    imgToWorldCoordsGivenHeight(pp,(double)img_width * 3.0 / 4.0, (double)img_height / 4.0, zA[1], &xA[1], &yA[1]);
    if(zA[1] - zA[0] == 0.0)
        return false;
    kxA = (xA[1] - xA[0]) / (zA[1] - zA[0]);
    kyA = (yA[1] - yA[0]) / (zA[1] - zA[0]);


    //Equation for line B
    imgToWorldCoordsGivenHeight(pp,(double)img_width / 4.0, (double)img_height * 3.0 / 4.0, zB[0], &xB[0], &yB[0]);
    imgToWorldCoordsGivenHeight(pp,(double)img_width / 4.0, (double)img_height * 3.0 / 4.0, zB[1], &xB[1], &yB[1]);
    if(zB[1] - zB[0] == 0.0)
        return false;
    kxB = (xB[1] - xB[0]) / (zB[1] - zB[0]);
    kyB = (yB[1] - yB[0]) / (zB[1] - zB[0]);

    //Use equations which define the intersection between both vision lines.
    if(kxA - kxB == 0.0)
        return false;
    *camz = ( xB[1] - zB[1]*kxB - xA[1] + zA[1]*kxA ) / (kxA - kxB);
    *camx = xA[1] + (*camz - zA[1])*kxA;
    *camy = yA[1] + (*camz - zA[1])*kyA;

    return true;
}


double SceneModel::getLimitHeightForProjectionLine(double x, double y) {
    double p20 = p_matrix[8],
           p21 = p_matrix[9],
           p22 = p_matrix[10],
           p23 = p_matrix[11];
    return -(p20 * x + p21 * y + p23) / p22;
}

double SceneModel::getLowestImageIntersectingHeight(int W, int H, double x, double y) {

    double p00 = p_matrix[0],
           p01 = p_matrix[1],
           p02 = p_matrix[2],
           p03 = p_matrix[3],
           p10 = p_matrix[4],
           p11 = p_matrix[5],
           p12 = p_matrix[6],
           p13 = p_matrix[7],
           p20 = p_matrix[8],
           p21 = p_matrix[9],
           p22 = p_matrix[10],
           p23 = p_matrix[11];

    double h, lowest = DBL_MAX;
    double
        xt = p00 * x + p01 * y + p03,
        yt = p10 * x + p11 * y + p13,
        t  = p20 * x + p21 * y + p23;

    //Check Y direction intersections
    //Lower_bound
    h = (yt  - H*t) / (H*p22 - p12);
    if(h > 0 && h < lowest)
        lowest = h;
    //Upper_bound
    h = - yt / p12;
    if(h > 0 && h < lowest)
        lowest = h;
    //Check X direction intersections
    //Lower_bound
    h = ( xt  - W*t ) / (W*p22 - p02);
    if(h > 0 && h < lowest)
        lowest = h;
    //Upper_bound
    h = - xt / p02;
    if(h > 0 && h < lowest)
        lowest = h;

    return lowest;

}

