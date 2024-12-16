#include "clear_checker.h"
#include <cmath>
#include <cstdlib>
#include <limits>
double centre_clear(const double x, const double y){
////Imagine four straight lines cutting centres of each line segment and numbering these clockwise from partition with obstacle
///// corner (-1 ,-1) as 1 .. 2 ,3 and 4. for eg (1,-1) corner lies in 4 
////checking the zone in which centre lies

if(y==0 || y<0){
    if(x==0 || x<0){

        ////zone 1
        return sqrt(std::pow(x + 1, 2) + std::pow(y + 1, 2));
    }
    else{
        ////zone 4
        return sqrt(std::pow(x - 1, 2) + std::pow(y + 1, 2));
    }
}
else{
    ////zone 2
    if(x==0 || x<0){
        return sqrt(std::pow(x + 1, 2) + std::pow(y - 1, 2));
    }
    /////zone 3
    else{
        return sqrt(std::pow(x - 1, 2) + std::pow(y - 1, 2));
    }

}
return 0.1;
}

double line_to_line(const Segment1 seg){
/////here we will check which is the closest edge of obstacle with which we should check line to line distance , others will be higher distance.
//////check angle of the line segment
////checking if line is parallel to y axis

////check line is complete on left of obstacle
if(seg.x0 < -1 && seg.x1 <-1){
    Segment1 segbase(-1,-1,-1,1);
    return kammkabanda(seg,segbase);
}
/////check line is completely on right of obstacle
if(seg.x0 >1 && seg.x1 >1){
    Segment1 segbase(1,-1,1,1);
    return kammkabanda(seg,segbase);
}
////right and left have already been covered . Pending is the one either at top or bottom
///since the each link is of length . if one end of link is above upper edge it can end below bottom edge y value and vice versa so
///checking of upper edge
if((seg.y0>1)|| ((seg.y1)>1)) {
    Segment1 segbase(-1,1,1,1);
    return kammkabanda(seg,segbase); 
}
///checking for lower edge
if((seg.y0<-1) || ((seg.y1)<-1)) {
    Segment1 segbase(1,-1,-1,-1); 
    return kammkabanda(seg,segbase); 
}
return 0.00002;
}



double pointDistance(double x1, double y1, double x2, double y2) {
    return sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

//this function checks gives the shortest distance between two lines
double kammkabanda(const Segment1 &seg1 , const Segment1 &segbase){
   double  x0=segbase.x0;
   double  x1=segbase.x1;
   double y0=segbase.y0;
   double y1=segbase.y1;
   double xp1=seg1.x0;
   double xp2=seg1.x1;
   double yp1=seg1.y0;
   double yp2=seg1.y1;
    double dist=std::numeric_limits<double>::max();
    /////check if the projection of points of line fall on line segment of obstacle
    ///if xp1 on base line
    double p1 = ((xp1-x0)*(x1-x0)+(yp1-y0)*(y1-y0))/((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0));
    ////projection on base line
    if(p1<0){
        dist=std::min(pointDistance(x0,y0,xp1,yp1),dist);
    }
    else if(p1>1){
        dist=std::min(pointDistance(x1,y1,xp1,yp1),dist); 
    }
    else{
        /////check what kind of line is the base line horizontal or vertical
        ////and since the direct projections are present we can directly get the values 
        if(x0==x1){
            dist=std::min(std::abs(xp1-x0),dist);
        }
        if(y0==y1){
            dist=std::min(std::abs(yp1-y0),dist); 
        }
    }
    ///checking for point 2 i.e xp2
   double p2 = ((xp2-x0)*(x1-x0)+(yp2-y0)*(y1-y0))/((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0));
    ////projection on base line

    if(p2<0){
        dist=std::min(pointDistance(x0,y0,xp2,yp2),dist);
    }
    else if(p2>1){
        dist=std::min (pointDistance(x1,y1,xp2,yp2),dist); 
    }
    else{
        /////check what kind of line is the base line horizontal or vertical
        ////and since the direct projections are present we can directly get the values 
        if(x0==x1){
            dist=std::min(std::abs(xp2-x0),dist);
        }
        if(y0==y1){
            dist=std::min(std::abs(yp2-y0),dist); 
        }
    }
   // Checking for base line's corners on link line
    double pb0 = ((x0 - xp1) * (xp2 - xp1) + (y0 - yp1) * (yp2 - yp1)) / ((xp2 - xp1) * (xp2 - xp1) + (yp2 - yp1) * (yp2 - yp1));
    if (pb0 < 0) {
        dist = std::min(pointDistance(x0, y0, xp1, yp1), dist);
    } else if (pb0 > 1) {
        dist = std::min(pointDistance(x0, y0, xp2, yp2), dist);
    } else {
        double cx = xp1 + pb0 * (xp2 - xp1);
        double cy = yp1 + pb0 * (yp2 - yp1);
        double d = pointDistance(cx, cy, xp1, yp1);
        double D = pointDistance(x0, y0, xp1, yp1);
        double tempd = sqrt((D * D) - (d * d)); // Avoiding negative values here
        dist = std::min(tempd, dist);
    }

    // Checking for the other point of base line on the link line
    double pb1 = ((x1 - xp1) * (xp2 - xp1) + (y1 - yp1) * (yp2 - yp1)) / ((xp2 - xp1) * (xp2 - xp1) + (yp2 - yp1) * (yp2 - yp1));
    if (pb1 < 0) {
        dist = std::min(pointDistance(x1, y1, xp1, yp1), dist);
    } else if (pb1 > 1) {
        dist = std::min(pointDistance(x1, y1, xp2, yp2), dist);
    } else {
        double cx = xp1 + pb1 * (xp2 - xp1);
        double cy = yp1 + pb1 * (yp2 - yp1);
        double d = pointDistance(cx, cy, xp1, yp1);
        double D = pointDistance(x1, y1, xp1, yp1);
        double tempd = sqrt((D * D) - (d * d)); // Avoiding negative values here
        dist = std::min(tempd, dist);
    }

    return dist; // Return the minimum distance found
}


double link_clearance(double x, double y, double theta0, double theta1, double theta2, double theta3, double theta4){
  ////creating segments 
  ////segments in the square bot 
  //////we know x'=xcosq-ysinq ,y'=xsinq+ycosq and x ,y will change as x,y +- 0.5
 /////left vertical  edge 
 double min_dist=std::numeric_limits<double>::max();
Segment1 seg_vl((-0.5)*cos(theta0)-(-0.5)*sin(theta0)+x,(-0.5)*sin(theta0)+(-0.5)*cos(theta0)+y,
                (-0.5)*cos(theta0)-(+0.5)*sin(theta0)+x,(-0.5)*sin(theta0)+(+0.5)*cos(theta0)+y);
 min_dist=std::min(line_to_line(seg_vl),min_dist);

 /////right vertical edge
 Segment1 seg_vr((+0.5)*cos(theta0)-(y-0.5)*sin(theta0)+x,(+0.5)*sin(theta0)+(-0.5)*cos(theta0)+y,
                (+0.5)*cos(theta0)-(+0.5)*sin(theta0)+x,(+0.5)*sin(theta0)+(+0.5)*cos(theta0)+y);
 min_dist=std::min(line_to_line(seg_vr),min_dist); 
 /////upper horizontal edge
 Segment1 seg_vu((+0.5)*cos(theta0)-(+0.5)*sin(theta0)+x,(+0.5)*sin(theta0)+(+0.5)*cos(theta0)+y,
                (-0.5)*cos(theta0)-(+0.5)*sin(theta0)+x,(-0.5)*sin(theta0)+(+0.5)*cos(theta0)+y);
 min_dist=std::min(line_to_line(seg_vu),min_dist); 

/////lower horizontal edge
Segment1 seg_vd((-0.5)*cos(theta0)-(-0.5)*sin(theta0)+x,(-0.5)*sin(theta0)+(-0.5)*cos(theta0)+y,
                (+0.5)*cos(theta0)-(-0.5)*sin(theta0)+x,(+0.5)*sin(theta0)+(-0.5)*cos(theta0)+y);
 min_dist=std::min(line_to_line(seg_vd),min_dist); 

//////now each link on kinematic chain will have two end points start & end 
////startx = x   then endx = xcostheta where theta is its angle with x axis
/////link 1 of kinematic chain
double l1_x1=x;
double l1_y1=y;
double l1_x2=l1_x1 + cos(theta1);
double l1_y2=l1_y1+sin(theta1);
Segment1 l1(l1_x1,l1_y1,l1_x2,l1_y2);
min_dist=std::min(line_to_line(l1),min_dist); 

////link2 of chain 
/////we know cordinates of end point of previous link will be start point for this link so dont define them
double l2_x2=(l1_x2)+cos(theta2);
double l2_y2=(l1_y2)+sin(theta2);
Segment1 l2(l1_x2,l1_y2,l2_x2,l2_y2);
min_dist=std::min(line_to_line(l2),min_dist); 

/////link 3 of chain
double l3_x2=(l2_x2)+cos(theta3);
double l3_y2=(l2_y2)+sin(theta3);
Segment1 l3(l2_x2,l2_y2,l3_x2,l3_y2);
min_dist=std::min(line_to_line(l3),min_dist); 

/////link 4 of chain
double l4_x2=(l3_x2)+cos(theta4);
double l4_y2=(l3_y2)+sin(theta4);
Segment1 l4(l3_x2,l3_y2,l4_x2,l4_y2);
min_dist=std::min(line_to_line(l4),min_dist); 

return min_dist;
}


