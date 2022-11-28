#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/registration/transformation_estimation_2D.h>


#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid_3d.h>




#include "nlohmann/json.hpp"

#include <vector>

#include <cmath>
#include <matplot/matplot.h>


#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include "icp/Normal2dEstimation.h"
int
pcl_run ( )
{
     int mode = 4;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr  norm_cloud(new pcl::PointCloud<pcl::Normal>);

    // Fill in the cloud data
    cloud->width  = 20;
    cloud->height = 1;
    cloud->resize (cloud->width * cloud->height);

    for (auto& point: *cloud)
    {
        point.x = 100 * rand () / (RAND_MAX + 1.0f);
        point.y = 100 * rand () / (RAND_MAX + 1.0f);
        point.z = 100 * rand () / (RAND_MAX + 1.0f);
    }

    if ( mode == 0){
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        // build the filter
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(0.8);
        outrem.setMinNeighborsInRadius (2);
        outrem.setKeepOrganized(false);
        // apply filter
        outrem.filter (*cloud_filtered);
    }
    else if (mode == 1){
        // build the condition
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
                                                                  pcl::ConditionAnd<pcl::PointXYZ> ());
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
                                                                                          pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
                                                                                          pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));
        // build the filter
        pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
        condrem.setCondition (range_cond);
        condrem.setInputCloud (cloud);
        condrem.setKeepOrganized(true);
        // apply filter
        condrem.filter (*cloud_filtered);
    }
    else if (mode == 2){

        // Create the filtering object
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (0.01f, 0.01f, 0.01f);
        sor.filter (*cloud_filtered);
    }
    else if (mode == 3){

        // Create the filtering object
        pcl::ApproximateVoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (0.01f, 0.01f, 0.01f);
        sor.filter (*cloud_filtered);
    }
    else if(mode == 4){


        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

        Normal2dEstimation norm_estim;
        norm_estim.setInputCloud(cloud);
        norm_estim.setSearchMethod (tree);

        norm_estim.setRadiusSearch (30);

        norm_estim.compute(norm_cloud);
        norm_estim.compute(cloud);



        std::cerr << "Cloud after norm_estim: " << std::endl;
        for (const auto& point: *norm_cloud)
            std::cerr << "    " << point.normal_x << " "
                      << point.normal_y << " "
                      << point.normal_z  << std::endl;

        pcl::PointCloud<pcl::PointNormal>::Ptr src(new pcl::PointCloud<pcl::PointNormal>);
        pcl::copyPointCloud(*cloud, *src);
        pcl::PointCloud<pcl::PointNormal>::Ptr tgt(new pcl::PointCloud<pcl::PointNormal>);
        pcl::copyPointCloud(*cloud, *tgt);

        norm_estim.setInputCloud(cloud);

        pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
        pcl::registration::TransformationEstimation2D<pcl::PointNormal, pcl::PointNormal>::Ptr est;
        est.reset(new pcl::registration::TransformationEstimation2D<pcl::PointNormal, pcl::PointNormal>);

        icp.setInputSource(src);
        icp.setInputTarget(tgt);
        // icp.setRANSACOutlierRejectionThreshold(ransac_par);
        icp.setRANSACIterations(20);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-3);
        pcl::PointCloud<pcl::PointNormal> output;

    }
    std::cerr << "Cloud before filtering: " << std::endl;
    for (const auto& point: *cloud)
        std::cerr << "    " << point.x << " "
                  << point.y << " "
                  << point.z << std::endl;
    // display pointcloud after filtering
    std::cerr << "Cloud after filtering: " << std::endl;
    for (const auto& point: *cloud_filtered)
        std::cerr << "    " << point.x << " "
                  << point.y << " "
                  << point.z << std::endl;
    return (0);
}

int plot_curv() {
    using namespace matplot;

    std::vector<double> x = logspace(-1, 2);
    std::vector<double> y = transform(x, [](auto x) { return pow(2, x); });
    loglog(x, y);

    show();
    return 0;
}

int plot_scatter4() {
    using namespace matplot;

    auto x = linspace(0, 3 * pi, 200);
    auto y = transform(x, [&](double x) { return cos(x) + rand(0, 1); });
    auto c = linspace(1, 10, x.size());

    auto l = scatter(x, y, 6, c);
    l->marker_face(true);

    show();
    return 0;
}


int plot_scatter7() {
    using namespace matplot;

    auto x = linspace(0, 3 * pi, 200);
    auto y = transform(x, [&](double x) { return cos(x) + rand(0, 1); });

    tiledlayout(2, 1);

    auto ax1 = nexttile();
    scatter(ax1, x, y);

    auto ax2 = nexttile();
    auto l = scatter(ax2, x, y);
    l->marker_face(true);
    l->marker_style(line_spec::marker_style::diamond);

    show();
    return 0;
}
int
main (int argc, char** argv)
{
    pcl_run( );

    return 1;
//    plot_scatter7();


    /*
function:

    json:
         file to json vector
         vector to json file
    pcl:
         create point cloud from vector
         approximate voxel grid filter
    octo map:
         points to map
         map to point cloud
    matplot:
        plot pointcloud in gnuplot












     raw point cloud => voxel grid filter


     */












    if (argc != 2)
    {
        std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
        exit(0);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    cloud->width  = 5;
    cloud->height = 1;
    cloud->resize (cloud->width * cloud->height);

    for (auto& point: *cloud)
    {
        point.x = 1024 * rand () / (RAND_MAX + 1.0f);
        point.y = 1024 * rand () / (RAND_MAX + 1.0f);
        point.z = 1024 * rand () / (RAND_MAX + 1.0f);
    }

    {
        using PointType = pcl::PointXYZ;
        pcl::RadiusOutlierRemoval<PointType> outrem;
        // build the filter
//        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(0.8);
        outrem.setMinNeighborsInRadius (2);
        outrem.setKeepOrganized(true);
        // apply filter
//        outrem.filter (*cloud_filtered);
    }


    {
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
        // build the filter
//        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(0.8);
        outrem.setMinNeighborsInRadius (2);
        outrem.setKeepOrganized(true);
        // apply filter
//        outrem.filter (*cloud_filtered);
    }


    if (strcmp(argv[1], "-r") == 0){
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        // build the filter
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(0.8);
        outrem.setMinNeighborsInRadius (2);
        outrem.setKeepOrganized(true);
        // apply filter
        outrem.filter (*cloud_filtered);
    }
    else if (strcmp(argv[1], "-c") == 0){
        // build the condition
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
                                                                  pcl::ConditionAnd<pcl::PointXYZ> ());
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
                                                                                          pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
                                                                                          pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));
        // build the filter
        pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
        condrem.setCondition (range_cond);
        condrem.setInputCloud (cloud);
        condrem.setKeepOrganized(true);
        // apply filter
        condrem.filter (*cloud_filtered);
    }
    else{
        std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
        exit(0);
    }
    std::cerr << "Cloud before filtering: " << std::endl;
    for (const auto& point: *cloud)
        std::cerr << "    " << point.x << " "
                  << point.y << " "
                  << point.z << std::endl;
    // display pointcloud after filtering
    std::cerr << "Cloud after filtering: " << std::endl;
    for (const auto& point: *cloud_filtered)
        std::cerr << "    " << point.x << " "
                  << point.y << " "
                  << point.z << std::endl;
    return (0);
}