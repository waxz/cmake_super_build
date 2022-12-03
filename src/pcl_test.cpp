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

#include "common/clock_time.h"

#include <vector>

#include <cmath>
#include <matplot/matplot.h>


#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>
#include <pcl/octree/octree_pointcloud_occupancy.h>


#include "icp/Normal2dEstimation.h"

namespace pcl{
    class Normal2dEstimation{

    public:
        void setRadius(){

        }
        void setTree(){

        }
        void setInputCloud(){

        }

        void compute(){

        }

    };
}



int
pcl_run ( )
{
     int mode = 5;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr  norm_cloud(new pcl::PointCloud<pcl::Normal>);

    // Fill in the cloud data
    cloud->width  = 500;
    cloud->height = 1;
    cloud->resize (cloud->width * cloud->height);

    float sample_radius = 3.0;
    float sample_angle_inc = 2*M_PI / cloud->width;
    float sample_angle = 0.0;
    for (auto& point: *cloud)
    {
        point.x = sample_radius * cos(sample_angle);
        point.y = sample_radius * sin(sample_angle);
        point.z = 0.0;
        sample_angle += sample_angle_inc;
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

    }else if(mode == 5){

        int point_num = cloud->size();


        float resolution = 0.05;

#if 0
        pcl::octree::OctreePointCloud<pcl::PointXYZ, pcl::octree::OctreeContainerPointIndices, pcl::octree::OctreeContainerEmpty,
                pcl::octree::Octree2BufBase<pcl::octree::OctreeContainerPointIndices, pcl::octree::OctreeContainerEmpty> > octree11(
                resolution);
        pcl::octree::OctreePointCloud<pcl::PointXYZ> otree_1(resolution);
        pcl::octree::OctreePointCloudPointVector<pcl::PointXYZ> otree_2(resolution);
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> otree_3 (resolution);

#endif


        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        pcl::search::KdTree<pcl::PointXYZ> kdtree_2;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

        octree.setInputCloud (cloud);
        octree.addPointsFromInputCloud ();

        kdtree.setInputCloud (cloud);
        kdtree_2.setInputCloud (cloud);



        pcl::PointXYZ searchPoint = cloud->at(0);

        // Neighbors within voxel search

        std::vector<int> pointIdxVec;

        std::vector<int> indices;
        std::vector<float> pointRadiusSquaredDistance;

        float radius = 0.1;

        std::cout << "check radiusSearch" << std::endl;

        int rt = 0;

        common::Time  t1 = common::FromUnixNow() ,t2= common::FromUnixNow();

        t1 = common::FromUnixNow();
        for(int i = 0 ; i < point_num; i++){
            searchPoint = cloud->at(i);
            rt = octree.radiusSearch (searchPoint, radius, indices, pointRadiusSquaredDistance);
        }
        t2= common::FromUnixNow();
        std::cout << "octree use time " << common::ToMicroSeconds(t2- t1) << " micro second" << std::endl;


        t1 = common::FromUnixNow();
        for(int i = 0 ; i < point_num; i++){
            searchPoint = cloud->at(i);
            rt = kdtree.radiusSearch (searchPoint, radius, indices, pointRadiusSquaredDistance);

        }
        t2= common::FromUnixNow();
        std::cout << "kdtree use time " << common::ToMicroSeconds(t2- t1) << " micro second" << std::endl;


        t1 = common::FromUnixNow();
        for(int i = 0 ; i < point_num; i++){
            searchPoint = cloud->at(i);
            rt = kdtree.radiusSearch (searchPoint, radius, indices, pointRadiusSquaredDistance);

        }
        t2= common::FromUnixNow();
        std::cout << "kdtree_2 use time " << common::ToMicroSeconds(t2- t1) << " micro second" << std::endl;



        t1 = common::FromUnixNow();

        using Scalar = float;
        auto& input_cloud = *cloud;



        for(int i = 0 ; i < point_num; i++){
            searchPoint = cloud->at( i);

            Eigen::Matrix<Scalar, 2, 2> covariance_matrix;
            Eigen::Matrix<Scalar, 4, 1> centroid;
            rt = kdtree_2.radiusSearch (searchPoint, radius, indices, pointRadiusSquaredDistance);

            {

                // Shifted data/with estimate of mean. This gives very good accuracy and good performance.
                // create the buffer on the stack which is much faster than using cloud[indices[i]] and centroid as a buffer
                Eigen::Matrix<Scalar, 1, 5, Eigen::RowMajor> accu = Eigen::Matrix<Scalar, 1, 5, Eigen::RowMajor>::Zero ();
                Eigen::Matrix<Scalar, 2, 1> K(0.0, 0.0);
#if 0
                for(const auto& index : indices){
                    if(pcl::isFinite(input_cloud[index])){
                        K.x() = input_cloud[index].x;
                        K.y() = input_cloud[index].y;
                        break;
                    }
                }
#endif

#if 1
                K.x() = input_cloud[indices[0]].x;
                K.y() = input_cloud[indices[0]].y;
#endif
                std::size_t point_count;
                point_count = indices.size ();
                for (const auto &index : indices)
                {
                    Scalar x = input_cloud[index].x - K.x(), y = input_cloud[index].y - K.y();
                    accu [0] += x * x;
                    accu [1] += x * y;
                    accu [2] += y * y;
                    accu [3] += x;
                    accu [4] += y;
                }

                if (point_count != 0)
                {
                    accu /= static_cast<Scalar> (point_count);
                    centroid[0] = accu[3] + K.x(); centroid[1] = accu[4] + K.y(); centroid[2] = 0.0;
                    centroid[3] = 1;
                    covariance_matrix.coeffRef (0) = accu [0] - accu [3] * accu [3];//xx
                    covariance_matrix.coeffRef (1) = accu [1] - accu [3] * accu [4];//xy
                    covariance_matrix.coeffRef (3) = accu [2] - accu [4] * accu [4];//yy
                    covariance_matrix.coeffRef (2) = covariance_matrix.coeff (1);//yx


                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig_solver(2);

                    eig_solver.compute(covariance_matrix);

                    Eigen::Vector2f eigen_values = eig_solver.eigenvalues();
                    Eigen::Matrix2f eigen_vectors = eig_solver.eigenvectors() ;
//                    std::cout << __LINE__ << "eigenvalues:\n" << eigen_values << std::endl;
//                    std::cout << __LINE__ << "eigenvectors:\n" << eigen_vectors << std::endl;

                    Eigen::MatrixX2f m(indices.size(),2);
                    int index = 0;
                    for(int j :  indices){
                        m(index,0) = input_cloud[j].x ;
                        m(index,1) = input_cloud[j].y ;
                        index++;
                    }

                    Eigen::VectorXf mean_vector = m.colwise().mean();

                    Eigen::MatrixXf centered = m.rowwise() - mean_vector.transpose();

                    Eigen::MatrixXf cov = (centered.adjoint() * centered) / ( m.rows() - 1 ) ;

//                    std::cout << "covariance_matrix:  \n" << covariance_matrix << std::endl;
//                    std::cout << "cov:  \n" << cov << std::endl;
//                    std::cout << "centered:  \n" << centered << std::endl;
//                    std::cout << "centroid:  \n" << centroid << std::endl;
//                    std::cout << "searchPoint: \n" << searchPoint << std::endl;

                    eig_solver.compute(cov);
                    eigen_values = eig_solver.eigenvalues();
                    eigen_vectors = eig_solver.eigenvectors() ;
//                    std::cout << __LINE__ << "eigenvalues:\n" << eigen_values << std::endl;
//                    std::cout << __LINE__ << "eigenvectors:\n" << eigen_vectors << std::endl;
                }
            }

        }
        t2= common::FromUnixNow();
        std::cout << "kdtree_2 norm use time " << common::ToMicroSeconds(t2- t1) << " micro second" << std::endl;



        int k = 20;
        pcl::Indices nn_indices (10);
        std::vector<float> nn_dists (10);
        t1 = common::FromUnixNow();
        Eigen::Matrix<Scalar, 2, 2> covariance_matrix;
        Eigen::Matrix<Scalar, 4, 1> centroid;

#pragma omp parallel for \
firstprivate(nn_indices, nn_dists)
        for(int i = 0 ; i < point_num; i++){
            rt = kdtree_2.radiusSearch ( cloud->at(i), radius, nn_indices, nn_dists);


            rt = kdtree_2.radiusSearch (searchPoint, radius, indices, pointRadiusSquaredDistance);

            {

                // Shifted data/with estimate of mean. This gives very good accuracy and good performance.
                // create the buffer on the stack which is much faster than using cloud[indices[i]] and centroid as a buffer
                Eigen::Matrix<Scalar, 1, 5, Eigen::RowMajor> accu = Eigen::Matrix<Scalar, 1, 5, Eigen::RowMajor>::Zero ();
                Eigen::Matrix<Scalar, 2, 1> K(0.0, 0.0);
#if 0
                for(const auto& index : indices){
                    if(pcl::isFinite(input_cloud[index])){
                        K.x() = input_cloud[index].x;
                        K.y() = input_cloud[index].y;
                        break;
                    }
                }
#endif

#if 1
                K.x() = input_cloud[indices[0]].x;
                K.y() = input_cloud[indices[0]].y;
#endif
                std::size_t point_count;
                point_count = indices.size ();
                for (const auto &index : indices)
                {
                    Scalar x = input_cloud[index].x - K.x(), y = input_cloud[index].y - K.y();
                    accu [0] += x * x;
                    accu [1] += x * y;
                    accu [2] += y * y;
                    accu [3] += x;
                    accu [4] += y;
                }

                if (point_count != 0)
                {
                    accu /= static_cast<Scalar> (point_count);
                    centroid[0] = accu[3] + K.x(); centroid[1] = accu[4] + K.y(); centroid[2] = 0.0;
                    centroid[3] = 1;
                    covariance_matrix.coeffRef (0) = accu [0] - accu [3] * accu [3];//xx
                    covariance_matrix.coeffRef (1) = accu [1] - accu [3] * accu [4];//xy
                    covariance_matrix.coeffRef (3) = accu [2] - accu [4] * accu [4];//yy
                    covariance_matrix.coeffRef (2) = covariance_matrix.coeff (1);//yx


                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig_solver(2);

                    eig_solver.compute(covariance_matrix);

                    Eigen::Vector2f eigen_values = eig_solver.eigenvalues();
                    Eigen::Matrix2f eigen_vectors = eig_solver.eigenvectors() ;
//                    std::cout << __LINE__ << "eigenvalues:\n" << eigen_values << std::endl;
//                    std::cout << __LINE__ << "eigenvectors:\n" << eigen_vectors << std::endl;

                    Eigen::MatrixX2f m(indices.size(),2);
                    int index = 0;
                    for(int j :  indices){
                        m(index,0) = input_cloud[j].x ;
                        m(index,1) = input_cloud[j].y ;
                        index++;
                    }

                    Eigen::VectorXf mean_vector = m.colwise().mean();

                    Eigen::MatrixXf centered = m.rowwise() - mean_vector.transpose();

                    Eigen::MatrixXf cov = (centered.adjoint() * centered) / ( m.rows() - 1 ) ;

//                    std::cout << "covariance_matrix:  \n" << covariance_matrix << std::endl;
//                    std::cout << "cov:  \n" << cov << std::endl;
//                    std::cout << "centered:  \n" << centered << std::endl;
//                    std::cout << "centroid:  \n" << centroid << std::endl;
//                    std::cout << "searchPoint: \n" << searchPoint << std::endl;

                    eig_solver.compute(cov);
                    eigen_values = eig_solver.eigenvalues();
                    eigen_vectors = eig_solver.eigenvectors() ;
//                    std::cout << __LINE__ << "eigenvalues:\n" << eigen_values << std::endl;
//                    std::cout << __LINE__ << "eigenvectors:\n" << eigen_vectors << std::endl;
                }
            }




        }
        t2= common::FromUnixNow();
        std::cout << "omp kdtree_2 norm use time " << common::ToMicroSeconds(t2- t1) << " micro second" << std::endl;



        if (rt> 0)
        {
            for (std::size_t i = 0; i < indices.size (); ++i)
                std::cout << "    "  <<   (*cloud)[ indices[i] ].x
                          << " " << (*cloud)[ indices[i] ].y
                          << " " << (*cloud)[ indices[i] ].z
                          << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
        }



    }
    std::cout << "Cloud before filtering: " << std::endl;
    for (const auto& point: *cloud)
        std::cout << "    " << point.x << " "
                  << point.y << " "
                  << point.z << std::endl;
    // display pointcloud after filtering
    std::cout << "Cloud after filtering: " << std::endl;
    for (const auto& point: *cloud_filtered)
        std::cout << "    " << point.x << " "
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
#ifdef _OPENMP
    std::cout << "use _OPENMP" <<std::endl;

#endif



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