/*
 * pcl_merge Point cloud merging. NaN values are filled with non-NaN values,
 *           then non-NaN values are averaged in position. Color is taken
 *           from the first non-NaN value.
 *  Created on: 14/04/2015
 *      Author: dfornas
 */
#ifndef PCLMERGE_H_
#define PCLMERGE_H_

#include <pm_tools/pcl_tools.h>

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/search/kdtree.h>

//Logging includes
#include <iostream>
#include <fstream>

#define CLOUD_SZ 307200 //640 x 480 resolution

template <typename PointT>
class CloudMerge
{

  //Point counters for each possible point in an organized point cloud with 640 x 480 resolution.
  // @ TODO Implement variable resolution
  int coeffs[CLOUD_SZ];
  float xvar[CLOUD_SZ], yvar[CLOUD_SZ], zvar[CLOUD_SZ];

  typename pcl::PointCloud<PointT>::Ptr c;

  /** Accumulate point b over our cloud, taking care of NaN values. */
  void accumPoints(PointT b, int idx);

public:

  CloudMerge(typename pcl::PointCloud<PointT>::Ptr p) : c(p){
	for (size_t i = 0; i < CLOUD_SZ; ++i){
		coeffs[i] = 0;
		xvar[i] = 0;
		yvar[i] = 0;
		zvar[i] = 0;
	}
  }

  /** Accumulate cloud b over a, filling gaps and averaging when necessary.  */
  void nanAwareOrganizedConcatenateMean(typename pcl::PointCloud<PointT>::Ptr b);

  /** Log the statistics of cloud merge process **/
  void logResults();

};


template<typename PointT>
void CloudMerge<PointT>::accumPoints(PointT b, int idx)
{
  c->points[idx].x = (c->points[idx].x * coeffs[idx] + b.x) / (coeffs[idx] + 1);
  c->points[idx].y = (c->points[idx].y * coeffs[idx] + b.y) / (coeffs[idx] + 1);
  c->points[idx].z = (c->points[idx].z * coeffs[idx] + b.z) / (coeffs[idx] + 1);
  xvar[idx] += b.x * b.x;
  yvar[idx] += b.y * b.y;
  zvar[idx] += b.z * b.z;
}

//Ejecuciones consecutivas asumen que vamos a acumular sobre a, segumarente sería mejor acumular sobre un miembro que se inicializa en el constructor.
template<typename PointT>
void CloudMerge<PointT>::nanAwareOrganizedConcatenateMean(typename pcl::PointCloud<PointT>::Ptr b)
{
  //A,B should be organized clouds of the same size. @ TODO Check and throw exception.
  for (size_t i = 0; i < c->points.size(); ++i){
    //First time a point is seen count=1;
    if(pcl::isFinite(c->points[i]) && coeffs[i] == 0){
      coeffs[i] = 1;
      xvar[i] = c->points[i].x * c->points[i].x;
      yvar[i] = c->points[i].y * c->points[i].y;
      zvar[i] = c->points[i].z * c->points[i].z;
    }
    if (pcl::isFinite(b->points[i])){
      if (!pcl::isFinite(c->points[i]))
      {
        //Point in B not found in A, add it.
        c->points[i] = b->points[i];
        //TODO: Search nearest neighbor color. Right now color is copied.
        xvar[i] = c->points[i].x * c->points[i].x;
        yvar[i] = c->points[i].y * c->points[i].y;
        zvar[i] = c->points[i].z * c->points[i].z;
        coeffs[i] = 1;
      }
      else
      {
        //Point found on both clouds. Weighted average.
        accumPoints(b->points[i], i); //c+d;//=(a->points[i]*coeffs[i]+b->points[i])/(coeffs[i]+1);
        coeffs[i]++;
      }
    }
  }

}

//The lo function is subject to change. Right now it groups by frequency with with its min, max and min variance.
template<typename PointT>
void CloudMerge<PointT>::logResults()
{
	/*
	  //Compute std dev for each axis
	  for (size_t i = 0; i < a->points.size(); ++i){
		  if( coeffs[i] > 0 ){
		  xvar[i] = sqrt(xvar[i] / (double)coeffs[i] - a->points[i].x * a->points[i].x);
		  yvar[i] = sqrt(yvar[i] / (double)coeffs[i] - a->points[i].y * a->points[i].y);
		  zvar[i] = sqrt(zvar[i] / (double)coeffs[i] - a->points[i].z * a->points[i].z);
		  }
	  }
	*/

	std::ofstream logfile;
	float values[45][4]; //45 será el numero de iteraciones, sería bueno guardarlo, pero ahora se itera fuera de la clase.
	//Las columnas son para cada valor: frecuencia var_max var_min var_mean  para la Z
	logfile.open ("/tmp/log_stats.txt");
	for(size_t i = 0; i < 45; ++i){
		values[i][0]=0;
		values[i][1]=0;
		values[i][2]=100;
		values[i][3]=0;
	}

	for (size_t i = 0; i < 307200; ++i){
		int value = coeffs[i];
		double var = zvar[i];//En Z
		if(zvar[i] > values[value][1]) values[value][1] = zvar[i];
		if(zvar[i] < values[value][2]) values[value][2] = zvar[i];
		values[value][3] = (values[value][0] * values[value][3] + zvar[i]) / (values[value][0]+1);
		values[value][0]++;
	}
	for(size_t i = 0; i < 307200; ++i){
			logfile << zvar[i] << std::endl;
	}
	/*
	for(size_t i = 0; i < 45; ++i){
		logfile << values[i][0] << " " << values[i][1] << " " << values[i][2] << " " << values[i][3] << std::endl;
	}*/
	logfile.close();
}

#endif
