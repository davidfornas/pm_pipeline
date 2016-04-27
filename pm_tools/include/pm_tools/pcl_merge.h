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
  float zstd_dev[CLOUD_SZ]; //Compute squared std dev for each pixel.This will only show Z changes.
  int iterations;

  typename pcl::PointCloud<PointT>::Ptr c;

  /** Accumulate point b over our cloud, taking care of NaN values. */
  void accumPoints(PointT b, int idx);

public:

  CloudMerge(typename pcl::PointCloud<PointT>::Ptr p) : c(p){
	for (size_t i = 0; i < CLOUD_SZ; ++i){
		coeffs[i] = 0;
		zstd_dev[i] = 0;
	}
	iterations = 1;
  }

  /** Accumulate cloud b over a, filling gaps and averaging when necessary.  */
  void nanAwareOrganizedConcatenateMean(typename pcl::PointCloud<PointT>::Ptr b);

  /** Log the statistics of cloud merge process **/
  void logResults();

};

//Accumulate points averaging, x,y should not vary that much, z is more variable so squared std dev is computed.
template<typename PointT>
void CloudMerge<PointT>::accumPoints(PointT b, int idx)
{
  int n = coeffs[idx] + 1;
  float x = c->points[idx].x, y = c->points[idx].y, z = c->points[idx].z;
  c->points[idx].x = (x * (n - 1) + b.x) / n;
  c->points[idx].y = (y * (n - 1) + b.y) / n;
  c->points[idx].z = (z * (n - 1) + b.z) / n;
  zstd_dev[idx] = sqrt(( (n - 1) * zstd_dev[idx] * zstd_dev[idx] + (b.z - z) * (b.z - c->points[idx].z)) / n);
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
      zstd_dev[i] = 0;
    }
    if (pcl::isFinite(b->points[i])){
      if (!pcl::isFinite(c->points[i]))
      {
        //Point in B not found in A, add it.
        c->points[i] = b->points[i];
        //TODO: Search nearest neighbor color. Right now color is copied.
        zstd_dev[i] = 0;
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
  iterations++;
}

//The lo function is subject to change. Right now it groups by frequency with with its min, max and min variance.
template<typename PointT>
void CloudMerge<PointT>::logResults()
{
	//Compute std dev for z axis
	for (size_t i = 0; i < CLOUD_SZ; ++i)
	  if( coeffs[i] > 0 )
		  zstd_dev[i] = sqrt(zstd_dev[i]);


	std::ofstream logfile;
	float values[45][4]; //45 será el numero de iteraciones, tengo iterations pero tendría que hacer la matriz dinámica.

	//Las columnas son para cada valor: frecuencia var_max var_min var_mean  para la Z
	logfile.open ("/tmp/log_stats.txt");
	for(size_t i = 0; i < iterations; ++i){
		values[i][0]=0;
		values[i][1]=0;
		values[i][2]=100;
		values[i][3]=0;
	}

	for (size_t i = 0; i < CLOUD_SZ; ++i){
		int value = coeffs[i];
		double var = zstd_dev[i];//En Z
		if(zstd_dev[i] > values[value][1]) values[value][1] = zstd_dev[i];
		if(zstd_dev[i] < values[value][2]) values[value][2] = zstd_dev[i];
		values[value][3] = (values[value][0] * values[value][3] + zstd_dev[i]) / (values[value][0]+1);
		values[value][0]++;
	}
	for(size_t i = 0; i < iterations; ++i){
		logfile << setiosflags(ios::fixed) << setprecision(2) << values[i][0] << " "
				<< setiosflags(ios::fixed) << setprecision(2) << values[i][1] << " "
				<< setiosflags(ios::fixed) << setprecision(2) << values[i][2] << " "
				<< setiosflags(ios::fixed) << setprecision(2) << values[i][3] << std::endl;
	}
	logfile.close();
}

#endif
