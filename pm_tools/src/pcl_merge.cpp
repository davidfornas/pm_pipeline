/*
 * pcl_merge
 * 			 
 *  Created on: 14/04/2015
 *      Author: dfornas
 */
#include <pm_tools/pcl_merge.h>


pcl::PointXYZRGB CloudMerge::accumPoints(pcl::PointXYZRGB a, pcl::PointXYZRGB b, int idx)
{
  pcl::PointXYZRGB c(a);
  c.x = (a.x * coeffs[idx] + b.x) / (coeffs[idx] + 1);
  c.y = (a.y * coeffs[idx] + b.y) / (coeffs[idx] + 1);
  c.z = (a.z * coeffs[idx] + b.z) / (coeffs[idx] + 1);
  xvar[idx] += b.x * b.x;
  yvar[idx] += b.y * b.y;
  zvar[idx] += b.z * b.z;
  return c;
}

void CloudMerge::nanAwareOrganizedConcatenateMean(pcl::PointCloud<pcl::PointXYZRGB>::Ptr a, pcl::PointCloud<pcl::PointXYZRGB>::Ptr b)
{
  //A,B should be organized clouds of the same size...
  for (size_t i = 0; i < a->points.size(); ++i){
    //First time a point is seen count=1;
    if(pcl::isFinite(a->points[i]) && coeffs[i] == 0){
      coeffs[i] = 1;
      xvar[i] = a->points[i].x * a->points[i].x;
      yvar[i] = a->points[i].y * a->points[i].y;
      zvar[i] = a->points[i].z * a->points[i].z;
    }

    if (pcl::isFinite(b->points[i])){
      if (!pcl::isFinite(a->points[i]))
      {
        //Point in B not found in A, add it.
        a->points[i] = b->points[i];
        //TODO: Search nearest neighbor color. Right now color is empty.
        xvar[i] = a->points[i].x * a->points[i].x;
        yvar[i] = a->points[i].y * a->points[i].y;
        zvar[i] = a->points[i].z * a->points[i].z;
        coeffs[i] = 1;
      }
      else
      {
        //Point found on both clouds. Weighted average.
        a->points[i] = accumPoints(a->points[i], b->points[i], i); //c+d;//=(a->points[i]*coeffs[i]+b->points[i])/(coeffs[i]+1);
        coeffs[i]++;
      }
    }
  }
}
