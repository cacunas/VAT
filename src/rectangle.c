/**
    @file        rectangle.c
 * @brief       rectangle type.
 * @author ORIAN team (orion@sophia.inria.fr)
 */     
 
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <cassert>

#include "common.h"
#include "rectangle.h"

namespace vsip {

rectangle_t 
new_rectangle()
{
    return (rectangle_t) calloc(1, sizeof(struct rectangle));
}

void 
free_rectangle(rectangle_t rect)
{
    if(rect != NULL)
	free(rect);
}


rectangle_t 
make_rectangle(int x, int y, int w, int h)
{
    rectangle_t rect;
    
    rect = new_rectangle();
    init_rectangle(rect, x, y, w, h);
    return rect;
}


void 
init_rectangle(rectangle_t rect, int x, int y, int w, int h)
{
    if (rect == NULL)
	return;
    RECT_XLEFT(rect) = x;
    RECT_XRIGHT(rect) = (w > 0 ? x + w - 1 : x);
    RECT_YTOP(rect) = y;
    RECT_YBOTTOM(rect) = (h > 0 ? y + h - 1 : y);
    RECT_WIDTH(rect) = w;
    RECT_HEIGHT(rect) = h;
}

void 
initRectangle(rectangle_t rect, int x, int y, int w, int h)
{
    if (rect == NULL)
	return;
    RECT_XLEFT(rect)  = x;
    RECT_XRIGHT(rect) = x + w - 1;
    RECT_YTOP(rect)   = y;
    RECT_YBOTTOM(rect)= y + h - 1;
    RECT_WIDTH(rect)  = w;
    RECT_HEIGHT(rect) = h;
}

rectangle_t copy_rectangle(rectangle_t rect)
{
    rectangle_t copy;
    
    copy = new_rectangle();
    memcpy(copy, rect, sizeof(struct rectangle));
    return copy;
}


rectangle_t 
absorb_rectangle(rectangle_t rect1, 
		 rectangle_t rect2)
{
    if(rect1 == NULL) return NULL;
    if(rect2 == NULL) return rect1;
    
    RECT_XLEFT(rect1)  = std::min(RECT_XLEFT(rect1), RECT_XLEFT(rect2));
    RECT_XRIGHT(rect1) = std::max(RECT_XRIGHT(rect1), RECT_XRIGHT(rect2));
    RECT_YTOP(rect1)   = std::min(RECT_YTOP(rect1), RECT_YTOP(rect2));
    RECT_YBOTTOM(rect1)= std::max(RECT_YBOTTOM(rect1), RECT_YBOTTOM(rect2));
    RECT_WIDTH(rect1)  = RECT_XRIGHT(rect1) - RECT_XLEFT(rect1) + 1;
    RECT_HEIGHT(rect1) = RECT_YBOTTOM(rect1)- RECT_YTOP(rect1) + 1;

    return rect1;
}

rectangle_t 
merge_rectangle(rectangle_t rect1, 
	        rectangle_t rect2)
{
    rectangle_t merge;

    if((rect1 == NULL) && (rect2 == NULL))
	return NULL;
    if(rect1 == NULL)
	return copy_rectangle(rect2);
    if(rect2 == NULL)
	return copy_rectangle(rect1);

    merge = new_rectangle();
    
    RECT_XLEFT(merge)  = std::min(RECT_XLEFT(rect1), RECT_XLEFT(rect2));
    RECT_XRIGHT(merge) = std::max(RECT_XRIGHT(rect1), RECT_XRIGHT(rect2));
    RECT_YTOP(merge)   = std::min(RECT_YTOP(rect1), RECT_YTOP(rect2));
    RECT_YBOTTOM(merge)= std::max(RECT_YBOTTOM(rect1), RECT_YBOTTOM(rect2));
    RECT_WIDTH(merge)  = RECT_XRIGHT(rect1) - RECT_XLEFT(rect1) + 1;
    RECT_HEIGHT(merge) = RECT_YBOTTOM(rect1)- RECT_YTOP(rect1) + 1;
    return merge;
}

rectangle_t 
rectangle_intersection(rectangle_t rect1, 
		       rectangle_t rect2)
{
  int x_left = std::max(RECT_XLEFT(rect1), RECT_XLEFT(rect2));
  int x_right = std::min(RECT_XRIGHT(rect1), RECT_XRIGHT(rect2));
  int y_top = std::max(RECT_YTOP(rect1), RECT_YTOP(rect2));
  int y_bot = std::min(RECT_YBOTTOM(rect1), RECT_YBOTTOM(rect2));

  if((x_left <= x_right) && (y_top <= y_bot)) 
    return make_rectangle(x_left, y_top, x_right - x_left + 1, y_bot - y_top + 1);
    else 
	return NULL;
}


rectangle_t 
rectangle_union(rectangle_t rect1, 
		rectangle_t rect2)
{
    int x_left = std::min(RECT_XLEFT(rect1), RECT_XLEFT(rect2));
    int x_right = std::max(RECT_XRIGHT(rect1), RECT_XRIGHT(rect2));
    int y_top = std::min(RECT_YTOP(rect1), RECT_YTOP(rect2));
    int y_bot = std::max(RECT_YBOTTOM(rect1), RECT_YBOTTOM(rect2));

    if((x_left <= x_right) && (y_top <= y_bot))
	return make_rectangle(x_left, 
			      y_top,
			      x_right - x_left + 1, 
			      y_bot - y_top + 1);
  
    else
	return NULL;
}

double  rectangle_intersect_ratio_strict(rectangle_t rec1, 
								         rectangle_t rec2)
{
  rectangle_t inter = NULL;
  double percent;
  
  if((rec1 == NULL) || (rec2) == NULL) return 0;
  inter = rectangle_intersection(rec1, rec2);
  if(inter == NULL) return 0;

  if( (((rec2->width)*(rec2->height))>((rec1->width)*(rec1->height))) || 
	  (((RECT_WIDTH(inter)*RECT_HEIGHT(inter)*1.0)/(RECT_WIDTH(rec2)*RECT_HEIGHT(rec2))) < 0.5) ){
	
	percent = (((RECT_WIDTH(rec2)*RECT_HEIGHT(rec2))-(RECT_WIDTH(inter)*RECT_HEIGHT(inter)))+100)/100;
	//+ 100 to be sure that percent is over 100 for classification.

	//percent = (RECT_WIDTH(rec2)*RECT_HEIGHT(rec2)*1.0)/
	 //(RECT_WIDTH(inter)*RECT_HEIGHT(inter));
  }
  else{
	percent = (RECT_WIDTH(inter)*RECT_HEIGHT(inter)*1.0)/
	  (RECT_WIDTH(rec1)*RECT_HEIGHT(rec1));
  }
  //if (percent > 1.0) percent = 1.0;
  free_rectangle(inter);
  return percent;
}       

double  rectangle_intersect_ratio(rectangle_t rec1, rectangle_t rec2) {
  rectangle_t inter = NULL;
  double percent;
  
  if((rec1 == NULL) || (rec2) == NULL) return 0;
  inter = rectangle_intersection(rec1, rec2);
  if(inter == NULL) return 0;
  
  percent = (double) (RECT_WIDTH(inter)*RECT_HEIGHT(inter)) / (double)(RECT_WIDTH(rec1)*RECT_HEIGHT(rec1));

  free_rectangle(inter);

  return percent;
}                          

/**
 * distance 2D of two rectangles. 
 * @author Nathanael ROTA (Nathanael.Rota@sophia.inria.fr)
 * @param rect1 1st rectangle.
 * @param rect2 2nd rectangle.
 * @return distance 2D of two rectangles. 
 */
int 
rectangle_distance(rectangle_t rect1, 
		   rectangle_t rect2)
{
  int xdelta = -1, ydelta = -1;
  int xl1, xr1, yt1, yb1;
  int xl2, xr2, yt2, yb2;

  if((rect1 == NULL) || (rect2 == NULL)) return -1;
  if(rect1 == rect2) return 0;
  
  xl1 = RECT_XLEFT(rect1);
  xr1 = RECT_XRIGHT(rect1);
  yt1 = RECT_YTOP(rect1);
  yb1 = RECT_YBOTTOM(rect1);
  xl2 = RECT_XLEFT(rect2);
  xr2 = RECT_XRIGHT(rect2);
  yt2 = RECT_YTOP(rect2);
  yb2 = RECT_YBOTTOM(rect2);

  if(!(ASCENDING(xl1, xr1, xl2, xr2)||(ASCENDING(xl2, xr2, xl1, xr1))))
    xdelta = 0;
  else if(ASCENDING(xl1, xr1, xl2, xr2))
    xdelta = (xl2 - xr1);
  else if(ASCENDING(xl2, xr2, xl1, xr1))
    xdelta = (xl1 - xr2);

  if(!(ASCENDING(yt1, yb1, yt2, yb2)||(ASCENDING(yt2, yb2, yt1, yb1)))) 
    ydelta = 0;
  else if(ASCENDING(yt1, yb1, yt2, yb2))
    ydelta = (yt2 - yb1); 
  else if(ASCENDING(yt2, yb2, yt1, yb1))
    ydelta = (yt1 - yb2); 

  return (int)sqrt((double) (xdelta*xdelta)+(ydelta*ydelta));
}

float rectangle_exact_distance(rectangle_t r1, 
			       rectangle_t r2){
  float xdelta, ydelta, grvX1, grvY1, grvX2, grvY2;
  
  if((r1 == NULL) || (r2 == NULL)) return -1;
  if(r1 == r2) return 0;
  
  grvX1 = (RECT_XRIGHT(r1) + RECT_XLEFT(r1))/2.0;
  grvY1 = (RECT_YTOP(r1) + RECT_YBOTTOM(r1))/2.0;
  grvX2 = (RECT_XRIGHT(r2) + RECT_XLEFT(r2))/2.0;
  grvY2 = (RECT_YTOP(r2) + RECT_YBOTTOM(r2))/2.0;
  
  xdelta = grvX1 - grvX2;
  ydelta = grvY1 - grvY2;

  return sqrt((xdelta*xdelta)+(ydelta*ydelta));
}

int
rectangle_in_rectangle(rectangle_t rect1,
		       rectangle_t rect2)
{
    return ( (RECT_XLEFT(rect1) >= RECT_XLEFT(rect2)) 	&&
	     (RECT_XRIGHT(rect1) <= RECT_XRIGHT(rect2)) &&
	     (RECT_YTOP(rect1) >= RECT_YTOP(rect2)) 	&&
	     (RECT_YBOTTOM(rect1) <= RECT_YBOTTOM(rect2)) );
}

int rectangleIntersectionArea(rectangle_t i_pRectangle1,rectangle_t i_pRectangle2)
{
	if ( (!i_pRectangle1) || (!i_pRectangle2) ) return 0;
	
	int xLeft = std::max(RECT_XLEFT(i_pRectangle1), RECT_XLEFT(i_pRectangle2));
	int xRight = std::min(RECT_XRIGHT(i_pRectangle1), RECT_XRIGHT(i_pRectangle2));
	int yTop = std::max(RECT_YTOP(i_pRectangle1), RECT_YTOP(i_pRectangle2));
	int yBot = std::min(RECT_YBOTTOM(i_pRectangle1), RECT_YBOTTOM(i_pRectangle2));
	int area = 0;
	if((xLeft <= xRight) && (yTop <= yBot)) 
	 area = (xRight - xLeft + 1)*(yBot - yTop + 1);
	return area;
}

int rectangleNIntersectionArea(const std::set<rectangle_t>& i_rectangles)
{
	std::set<rectangle_t>::iterator firstRectangle = i_rectangles.begin();
	
	int xLeft = (*firstRectangle)->xleft;
	int xRight = (*firstRectangle)->xright;
	int yTop = (*firstRectangle)->ytop;
	int yBot = (*firstRectangle)->ybottom;
	firstRectangle++;
			
	std::set<rectangle_t>::iterator rectangleIter;
	for (rectangleIter = firstRectangle; rectangleIter != i_rectangles.end(); rectangleIter++)
	{
		xLeft = std::max(xLeft,(*rectangleIter)->xleft);
		xRight = std::min(xRight, (*rectangleIter)->xright);
		yTop = std::max(yTop, (*rectangleIter)->ytop);
		yBot = std::min(yBot, (*rectangleIter)->ybottom);
	}
	
	int area = 0;
	if((xLeft <= xRight) && (yTop <= yBot))
		area = (xRight - xLeft + 1)*(yBot - yTop + 1);
	return area;
}

int rectangleArea(rectangle_t i_pRectangle)
{
	int area = i_pRectangle->width*i_pRectangle->height;
	return area;
}

frectangle_t new_frectangle() {
  return (frectangle_t) calloc(1, sizeof(struct frectangle));
}

void free_frectangle(frectangle_t rect) {

    if(rect != NULL)
	free(rect);

}

frectangle_t make_frectangle(double x, double y, double w, double h) {
    frectangle_t rect;
    
    rect = new_frectangle();
    init_frectangle(rect, x, y, w, h);
    return rect;
}

void init_frectangle(frectangle_t rect, double x, double y, double w, double h) {
    if (rect == NULL)
	return;
    RECT_XLEFT(rect) = x;
    RECT_XRIGHT(rect) = (w > 0 ? x + w - 1 : x);
    RECT_YTOP(rect) = y;
    RECT_YBOTTOM(rect) = (h > 0 ? y + h - 1 : y);
    RECT_WIDTH(rect) = w;
    RECT_HEIGHT(rect) = h;
}

int rectangles_intersect(rectangle_t r1, rectangle_t r2) {

  if(RECT_XLEFT(r1) >= RECT_XRIGHT(r2) || RECT_XLEFT(r2) >= RECT_XRIGHT(r1))
    return 0;

  if(RECT_YTOP(r1) >= RECT_YBOTTOM(r2) || RECT_YTOP(r2) >= RECT_YBOTTOM(r1))
    return 0;

  return 1;

}

double rect_distance(rectangle_t r1, rectangle_t r2) {
  if(rectangles_intersect(r1, r2))
    return 0.0;

  int dX, dY;

  if(RECT_XLEFT(r1) >= RECT_XRIGHT(r2)) //r1 to the right of r2
    dX = RECT_XLEFT(r1) - RECT_XRIGHT(r2);
  else if(RECT_XLEFT(r2) >= RECT_XRIGHT(r1)) //r2 to the right of r1
    dX = RECT_XLEFT(r2) - RECT_XRIGHT(r1);
  else //Intersection in x dimension exists, so distance in this dimension is 0.
    dX = 0;

  if(RECT_YTOP(r1) >= RECT_YBOTTOM(r2)) //r1 at the bottom of r2
    dY = RECT_YTOP(r1) - RECT_YBOTTOM(r2);
  else if(RECT_YTOP(r2) >= RECT_YBOTTOM(r1)) //r2 at the bottom of r1
    dY = RECT_YTOP(r2) - RECT_YBOTTOM(r1);
  else //Intersection in y dimension exists, so distance in this dimension is 0.
    dY = 0;

  if(dX == 0) return dY;
  if(dY == 0) return dX;

  return sqrt(dX*dX + dY*dY);
}


} // end namespace vsip

