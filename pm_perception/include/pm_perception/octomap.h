/*
 * octomap This class is  wrapper over the octomap library
 *
 *  Created on: 20/05/2016
 *      Author: dfornas
 */
#ifndef OCTOMAPWRAP_H_
#define OCTOMAPWRAP_H_

#include <ros/ros.h>

//To remove
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/pca.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pm_tools/pcl_tools.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

//To remove
using namespace std;
using namespace octomap;

template <typename PointT>
class OctomapManager
{
  typedef typename pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::iterator CloudIt;

  CloudPtr cloud_;

public:

  void print_query_info(point3d query, OcTreeNode* node) {
    if (node != NULL) {
      cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
    }
    else
      cout << "occupancy probability at " << query << ":\t is unknown" << endl;
  }

  OctomapManager(CloudPtr in_cloud) :
      cloud_(in_cloud)
  {
  }

  OctomapManager(std::string in_cloud_file)
  {
	  PCLTools<PointT>::cloudFromPCD(cloud_, in_cloud_file);
  }


  /** Display result  */
  void display();
  void do_stuff();

};

template<typename PointT>
void OctomapManager<PointT>::display()
{
	pcl::visualization::PCLVisualizer viewer("Cluster measure viewer");
    viewer.addPointCloud(cloud_);
    viewer.spin();
}


template<typename PointT>
void OctomapManager<PointT>::do_stuff(){

  cout << endl;
  cout << "generating example map" << endl;

  OcTree tree (0.1);  // create empty tree with resolution 0.1


  // insert some measurements of occupied cells

  for (int x=-20; x<20; x++) {
    for (int y=-20; y<20; y++) {
      for (int z=-20; z<20; z++) {
        point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
        tree.updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }

  // insert some measurements of free cells

  for (int x=-30; x<30; x++) {
    for (int y=-30; y<30; y++) {
      for (int z=-30; z<30; z++) {
        point3d endpoint ((float) x*0.02f-1.0f, (float) y*0.02f-1.0f, (float) z*0.02f-1.0f);
        tree.updateNode(endpoint, false);  // integrate 'free' measurement
      }
    }
  }

  cout << endl;
  cout << "performing some queries:" << endl;

  point3d query (0., 0., 0.);
  OcTreeNode* result = tree.search (query);
  print_query_info(query, result);

  query = point3d(-1.,-1.,-1.);
  result = tree.search (query);
  print_query_info(query, result);

  query = point3d(1.,1.,1.);
  result = tree.search (query);
  print_query_info(query, result);


  cout << endl;
  tree.writeBinary("simple_tree.bt");
  cout << "wrote example file simple_tree.bt" << endl << endl;
  cout << "now you can use octovis to visualize: octovis simple_tree.bt"  << endl;
  cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl  << endl;

}

#endif
