 /*
  * Software License Agreement (BSD License)
  *
  *  Point Cloud Library (PCL) - www.pointclouds.org
  *  Copyright (c) 2011, Willow Garage, Inc.
  *
  *  All rights reserved.
  *
  *  Redistribution and use in source and binary forms, with or without
  *  modification, are permitted provided that the following conditions
  *  are met:
  *
  *   * Redistributions of source code must retain the above copyright
  *     notice, this list of conditions and the following disclaimer.
  *   * Redistributions in binary form must reproduce the above
  *     copyright notice, this list of conditions and the following
  *     disclaimer in the documentation and/or other materials provided
  *     with the distribution.
  *   * Neither the name of the copyright holder(s) nor the names of its
  *     contributors may be used to endorse or promote products derived
  *     from this software without specific prior written permission.
  *
  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  *  POSSIBILITY OF SUCH DAMAGE.
  *
  * $Id$
  *
  */
  
#include <pcl/filters/crop_hull.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/common/common.h>

#include <pcl/filters/extract_indices.h> // for ExtractIndices
#include <pcl/segmentation/extract_clusters.h> // for EuclideanClusterExtraction
#include <pcl/filters/voxel_grid.h> // for VoxelGrid


#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid_3d.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

using PointT = PointXYZ;
using CloudT = PointCloud<PointT>;

const static double default_alpha = 1e3f;

static void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s hull_cloud.pcd input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -alpha X = the hull alpha value (0+) (default: ");
  print_value ("%f", default_alpha);
  print_info (")\n");
}

static bool
loadCloud (std::string const& filename, CloudT &cloud)
{
  TicToc tt;
  print_highlight ("Loading ");
  print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud) < 0)
    return (false);

  print_info ("[done, ");
  print_value ("%g", tt.toc ());
  print_info (" ms : ");
  print_value ("%d", cloud.width * cloud.height);
  print_info (" points]\n");
  print_info ("Available dimensions: ");
  print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

static void
saveCloud (std::string const& filename, CloudT const& cloud)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving ");
  print_value ("%s ", filename.c_str ());

  pcl::io::savePCDFile (filename, cloud);

  print_info ("[done, ");
  print_value ("%g", tt.toc ());
  print_info (" ms : ");
  print_value ("%d", cloud.width * cloud.height);
  print_info (" points]\n");
}

static void
cropToHull (CloudT::Ptr output, CloudT::Ptr input, CloudT::Ptr hull_cloud, std::vector<pcl::Vertices> const& polygons, int dim)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Cropping ");

  CropHull<PointT> crop_filter;
//    crop_filter.setNegative(true);
//    crop_filter.setNegative(false);
  crop_filter.setCropOutside(false);
  crop_filter.setInputCloud (input);
  crop_filter.setHullCloud (hull_cloud);
  crop_filter.setHullIndices (polygons);
  crop_filter.setDim (dim);


    crop_filter.filter (*output);

  print_info ("[done, ");
  print_value ("%g", tt.toc ());
  print_info (" ms : ");
  print_value ("%d", output->size());
  print_info (" points passed crop]\n");
    print_info ("crop_filter.getNegative, %d\n", crop_filter.getNegative());


}

static CloudT::Ptr
calculateHull (std::vector<pcl::Vertices>& polygons, int& dim, CloudT::Ptr cloud, double alpha)
{
  pcl::ConcaveHull<PointT> hull_calculator;
  CloudT::Ptr hull (new CloudT);
  hull_calculator.setInputCloud (cloud);
  hull_calculator.setAlpha (alpha);
  hull_calculator.reconstruct (*hull, polygons);
  
  dim = hull_calculator.getDimension ();
  return hull;
}

int
main (int argc, char** argv)
{

#if 0
    print_info ("Filter a point cloud using the convex hull of another point "
              "cloud. For more information, use: %s -h\n", argv[0]);

  if (argc < 4)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (p_file_indices.size () != 3)
  {
    print_error ("Need at least three pcd files to continue.\n");
    return (-1);
  }

  // Command line parsing
  double alpha = default_alpha;
  parse_argument (argc, argv, "-alpha", alpha);
#endif


  CloudT::Ptr hull_cloud (new CloudT);
  CloudT::Ptr hull_points (new CloudT);
    CloudT::Ptr triangle_points (new CloudT);

    CloudT::Ptr input_cloud (new CloudT);
  CloudT::Ptr output_cloud (new CloudT);
  std::vector<pcl::Vertices> hull_polygons;
    std::vector<pcl::Vertices> triangle_polygons;

  int dim = 0;  
#if 0
    if (!loadCloud (argv[p_file_indices[0]], *hull_cloud))
    return (-1);

  if (!loadCloud (argv[p_file_indices[1]], *input_cloud))
    return (-1);

  hull_points = calculateHull (hull_polygons, dim, hull_cloud, alpha);

#endif


    {
        // hull crop demo
        size_t width = 100;
        size_t height = 100;

        float resolution = 0.1;


        input_cloud->points.resize(width*height);
        for(size_t i = 0 ; i < height;i++){
            for(size_t j = 0 ; j < width; j++){
                input_cloud->points[ i*width + j].x = i*resolution;
                input_cloud->points[ i*width + j].y = j*resolution;
                input_cloud->points[ i*width + j].z = 0.0;

            }
        }

        input_cloud->width = input_cloud->points.size();
        input_cloud->height = 1;


        hull_points->points.resize(4);
        hull_points->points[0].x = -1;
        hull_points->points[0].y = -1;
        hull_points->points[0].z = 0.0;

        hull_points->points[1].x = 1;
        hull_points->points[1].y = -1;
        hull_points->points[1].z = 0.0;

        hull_points->points[2].x = 15.0;
        hull_points->points[2].y = 14.0;
        hull_points->points[2].z = 0.0;


        hull_points->points[3].x = 14;
        hull_points->points[3].y = 16;
        hull_points->points[3].z = 0.0;

        triangle_points->points.resize(3);

        triangle_points->points[0].x = 5.0 -1e-3;
        triangle_points->points[0].y = 5.0-1e-3;
        triangle_points->points[0].z = 0.0;

        triangle_points->points[1].x = 5.0 + 2.1;
        triangle_points->points[1].y = 5.0 + 1.8;
        triangle_points->points[1].z = 0.0;

        triangle_points->points[2].x = 5.0 + 2.0;
        triangle_points->points[2].y = 5.0 + 2.0;
        triangle_points->points[2].z = 0.0;

        std::vector<std::vector<pcl::index_t>> box_elements = {
                {0, 1, 2}, // l
                {1, 2, 3}, // l
                {2, 3, 0}, // f
                {3, 0, 1}};

        std::vector<std::vector<pcl::index_t>> triangle_elements = {
                {0, 1, 2}, // l
                {1, 2, 0}, // l
                {2, 0, 1}};
        hull_polygons.resize(4);
        for (size_t i = 0; i < 4; ++i) {
            hull_polygons[i].vertices = box_elements[i];
        }

        triangle_polygons.resize(3);
        for (size_t i = 0; i < 3; ++i) {
            triangle_polygons[i].vertices = triangle_elements[i];
        }


        dim = 2;

        cropToHull (output_cloud, input_cloud, triangle_points, triangle_polygons, dim);


        cropToHull (output_cloud, input_cloud, hull_points, hull_polygons, dim);


        std::cout << "input_cloud size = " << input_cloud->size() << std::endl;

        std::cout << "output_cloud size = " << output_cloud->size() << std::endl;


        if (!output_cloud->empty ()){

            saveCloud ( "crop.pcd", *output_cloud);
            saveCloud ( "before_crop.pcd", *input_cloud);

        }
        else
            print_error ("No points passed crop.\n");

    }




    {
        // segmentation demo
        pcl::PCDWriter writer;
        std::string outfile = "segment_out.pcd";


        ///segment scene into clusters with given distance tolerance using euclidean clustering
        double segradius = 0.2;


        //Filter by magnitude
        std::cout << "Clustering using EuclideanClusterExtraction with tolerance <= "<< segradius <<  "..." << std::endl;

        pcl::search::KdTree<PointT>::Ptr segtree (new pcl::search::KdTree<PointT>);
        segtree->setInputCloud (output_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;

        ec.setClusterTolerance (segradius);
        ec.setMinClusterSize (50);
        ec.setMaxClusterSize (100000);
        ec.setSearchMethod (segtree);
        ec.setInputCloud (output_cloud);
        ec.extract (cluster_indices);

        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
        {
            pcl::PointCloud<PointT>::Ptr cloud_cluster_don (new pcl::PointCloud<PointT>);
            for (const auto &index : it->indices){
                cloud_cluster_don->points.push_back ((*output_cloud)[index]);
            }

            cloud_cluster_don->width = cloud_cluster_don->size ();
            cloud_cluster_don->height = 1;
            cloud_cluster_don->is_dense = true;

            pcl::PointXYZ minPt, maxPt;
            pcl::getMinMax3D (*cloud_cluster_don, minPt, maxPt);
            std::cout << "Max x: " << maxPt.x << std::endl;
            std::cout << "Max y: " << maxPt.y << std::endl;
            std::cout << "Max z: " << maxPt.z << std::endl;
            std::cout << "Min x: " << minPt.x << std::endl;
            std::cout << "Min y: " << minPt.y << std::endl;
            std::cout << "Min z: " << minPt.z << std::endl;


            std::cout << "PointCloud representing the Cluster: " << cloud_cluster_don->size () << " data points." << std::endl;
            std::stringstream ss;
            ss << outfile.substr(0,outfile.length()-4) << "_cluster_" << j << ".pcd";
            writer.write<PointT> (ss.str (), *cloud_cluster_don, false);
        }

    }

    {
#if 0
        using PointType = pcl::PointXYZ;

        pcl::IterativeClosestPointNonLinear<PointType, PointType> icp;

        pcl::registration::WarpPointRigid3D<PointType, PointType>::Ptr warp_fcn
                (new pcl::registration::WarpPointRigid3D<PointType, PointType>);

        // Create a TransformationEstimationLM object, and set the warp to it
        pcl::registration::TransformationEstimationLM<PointType, PointType>::Ptr te (new pcl::registration::TransformationEstimationLM<PointType, PointType>);
        te->setWarpFunction (warp_fcn);

        // Pass the TransformationEstimation objec to the ICP algorithm
        icp.setTransformationEstimation (te);

        icp.setInputTarget (model);

        icp.setInputSource (data);

        CloudPtr tmp (new Cloud);
        icp.align (*tmp);
#endif



    }


  return (0);
}


