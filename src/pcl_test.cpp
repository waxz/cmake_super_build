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


#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/sac_model_circle.h>





#include "icp/Normal2dEstimation.h"

#define CHECK_IS_SAME_TYPE(T1, T2) typename std::enable_if_t< std::is_same<T1, T2>::value, bool> = true


namespace pcl{
    /*
             pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        pcl::search::KdTree<pcl::PointXYZ> kdtree_2;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

     */

    template<typename PointT>
    void createTree(typename pcl::PointCloud<PointT>::ConstPtr t_input_cloud, pcl::KdTreeFLANN<PointT>& t_tree){
        t_tree.setInputCloud(t_input_cloud);
    }

    template<typename PointT>
    void createTree(typename pcl::PointCloud<PointT>::ConstPtr t_input_cloud, pcl::search::KdTree<PointT>& t_tree){
        t_tree.setInputCloud(t_input_cloud);
    }

    template<typename PointT>
    void createTree(typename pcl::PointCloud<PointT>::ConstPtr t_input_cloud, pcl::octree::OctreePointCloudSearch<PointT>& t_tree){
        t_tree.setInputCloud(t_input_cloud);
        t_tree.addPointsFromInputCloud ();
    }



    struct NormalEst2d{
        using Scalar = float;
        Eigen::Matrix<Scalar, 1, 5, Eigen::RowMajor> accu = Eigen::Matrix<Scalar, 1, 5, Eigen::RowMajor>::Zero ();
        Eigen::Matrix<Scalar, 2, 1> K{0.0, 0.0};
        Eigen::Matrix<Scalar, 4, 1> centroid;
        Eigen::Matrix<Scalar, 2, 2> covariance_matrix;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig_solver{2};
        Eigen::Matrix<Scalar, 2, 1> view_point{0.0, 0.0};

        int point_count = 0;

        void reset(){
            point_count = 0;
            accu = Eigen::Matrix<Scalar, 1, 5, Eigen::RowMajor>::Zero ();
        }

        void addCenter(float x, float y){

            K.x() = x;
            K.y() = y;
        }
        void addPoints(float x, float y){
            x -= K.x();
            y -= K.y();

            accu [0] += x * x;
            accu [1] += x * y;
            accu [2] += y * y;
            accu [3] += x;
            accu [4] += y;
            point_count ++;
        }

        void compute(float& normal_x,float& normal_y,float& normal_z,float& curvature ){

            if(point_count <=3){
                normal_x  = normal_y  = normal_z  =  curvature = std::numeric_limits<float>::quiet_NaN ();

                return;
            }
            accu /= static_cast<Scalar> (point_count);
            centroid[0] = accu[3] + K.x(); centroid[1] = accu[4] + K.y(); centroid[2] = 0.0;
            centroid[3] = 1;
            covariance_matrix.coeffRef (0) = accu [0] - accu [3] * accu [3];//xx
            covariance_matrix.coeffRef (1) = accu [1] - accu [3] * accu [4];//xy
            covariance_matrix.coeffRef (3) = accu [2] - accu [4] * accu [4];//yy
            covariance_matrix.coeffRef (2) = covariance_matrix.coeff (1);//yx

            eig_solver.compute(covariance_matrix);

            auto& eigen_values = eig_solver.eigenvalues();
            auto& eigen_vectors = eig_solver.eigenvectors() ;

            normal_x = eigen_vectors(0,0);
            normal_y = eigen_vectors(1,0);
            normal_z = 0.0;

            Eigen::Matrix <float, 2, 1> normal (normal_x, normal_y);
            Eigen::Matrix <float, 2, 1> vp ( view_point.x() - K.x(), view_point.y() - K.y());
            normal.normalize();
            vp.normalize();

            // Dot product between the (viewpoint - point) and the plane normal
            float cos_theta = vp.dot (normal);
            // Flip the plane normal
            if (cos_theta < 0)
            {
                normal_x *= -1;
                normal_y *= -1;
            }
//            std::cout <<__FILE__ <<  __LINE__  << " normal_x: " << normal_x  << " normal_y: " << normal_y
//            << " cos_theta: " << cos_theta << std::endl;

            curvature = std:: acos(std::abs(cos_theta));

//            std::cout <<__FILE__ <<  __LINE__  << " curvature: " << curvature << std::endl;


        }


    };

    template<typename PointT, typename TreeType>
    class Normal2dEstimation{

    public:
        using PointCloud = pcl::PointCloud<PointT>;
        using PointCloudPtr = typename PointCloud::Ptr;
        using PointCloudConstPtr = typename PointCloud::ConstPtr;
        using Scalar = float;

    private:
        TreeType m_tree;
        float m_query_radius = 0.1;
        PointCloudConstPtr m_input_cloud;

        std::vector<int> m_query_indices;
        std::vector<float> m_query_distance;
        NormalEst2d m_NormalEst2d;

    public:
        Normal2dEstimation(const TreeType& t_tree, float radius = 0.1, int num = 20):
        m_tree(t_tree),
        m_query_radius(radius),
        m_query_indices(num),
        m_query_distance(num){

        }

        void setInputCloud(const PointCloudConstPtr& t_input_cloud)
        {
            m_input_cloud = t_input_cloud;
            createTree(m_input_cloud,m_tree );
        }

        void setRadius(){

        }
        void setTree(){

        }
        void setInputCloud(){

        }


        void compute( const pcl::PointCloud<pcl::PointNormal>::Ptr&  output){

            int input_point_num = m_input_cloud->points.size();
            output->points.resize(input_point_num,pcl::PointNormal(0.0,0.0,0.0,0.0,0.0,0.0));
            output->height = m_input_cloud->height;
            output->width = m_input_cloud->width;
            output->is_dense = true;

            int rt = 0;
            Eigen::Matrix<Scalar, 1, 5, Eigen::RowMajor> accu = Eigen::Matrix<Scalar, 1, 5, Eigen::RowMajor>::Zero ();
            Eigen::Matrix<Scalar, 2, 1> K(0.0, 0.0);
            Eigen::Matrix<Scalar, 4, 1> centroid;
            Eigen::Matrix<Scalar, 2, 2> covariance_matrix;
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig_solver(2);

            for(int i = 0 ; i < input_point_num;i++){


                auto& query_point = m_input_cloud->at(i);
                rt = m_tree.radiusSearch (query_point, m_query_radius, m_query_indices, m_query_distance);

                std::size_t point_count;
                point_count = m_query_indices.size ();
                if(point_count <=3){
                    output->points[i].normal_x  = output->points[i].normal_y  = output->points[i].normal_z  = output->points[i].curvature = std::numeric_limits<float>::quiet_NaN ();
                    continue;
                }


                K.x() = m_input_cloud->at(m_query_indices[0]).x;
                K.y() = m_input_cloud->at(m_query_indices[0]).y;


                for (const auto &index : m_query_indices)
                {
                    Scalar x = m_input_cloud->at(index).x - K.x(), y = m_input_cloud->at(index).y - K.y();
                    accu [0] += x * x;
                    accu [1] += x * y;
                    accu [2] += y * y;
                    accu [3] += x;
                    accu [4] += y;
                }


                {
                    accu /= static_cast<Scalar> (point_count);
                    centroid[0] = accu[3] + K.x(); centroid[1] = accu[4] + K.y(); centroid[2] = 0.0;
                    centroid[3] = 1;
                    covariance_matrix.coeffRef (0) = accu [0] - accu [3] * accu [3];//xx
                    covariance_matrix.coeffRef (1) = accu [1] - accu [3] * accu [4];//xy
                    covariance_matrix.coeffRef (3) = accu [2] - accu [4] * accu [4];//yy
                    covariance_matrix.coeffRef (2) = covariance_matrix.coeff (1);//yx



                    eig_solver.compute(covariance_matrix);
#if 0
                    {
                        Eigen::MatrixX2f m(m_query_indices.size(),2);
                        int index = 0;
                        for(int j :  m_query_indices){
                            m(index,0) = m_input_cloud->at(j).x ;
                            m(index,1) = m_input_cloud->at(j).y ;
                            index++;
                        }

                        Eigen::VectorXf mean_vector = m.colwise().mean();

                        Eigen::MatrixXf centered = m.rowwise() - mean_vector.transpose();

                        Eigen::MatrixXf cov = (centered.adjoint() * centered) / ( m.rows() - 1 ) ;
                        eig_solver.compute(cov);

                    }
#endif






                    auto& eigen_values = eig_solver.eigenvalues();
                    auto& eigen_vectors = eig_solver.eigenvectors() ;
                    auto& nx =  output->points[i].normal_x;
                    auto& ny =  output->points[i].normal_y;
                    auto& nz =  output->points[i].normal_z;
                    nx = eigen_vectors(0,0);
                    ny = eigen_vectors(1,0);
#if 0
                    if(std::abs(eigen_values(0)) < std::abs(eigen_values(1))){
                        nx = eigen_vectors(0,0);
                        ny = eigen_vectors(1,0);
                    }else{
                        nx = eigen_vectors(0,1);
                        ny = eigen_vectors(1,1);
                    }
#endif
                    nz = 0.0;

                    Eigen::Matrix <float, 2, 1> normal (nx, ny);
                    Eigen::Matrix <float, 2, 1> vp ( - query_point.x, - query_point.y);

                    // Dot product between the (viewpoint - point) and the plane normal
                    float cos_theta = vp.dot (normal);
                    // Flip the plane normal
                    if (cos_theta < 0)
                    {
                        nx *= -1;
                        ny *= -1;
                    }


//                    std::cout << __LINE__ << "eigenvalues:\n" << eigen_values << std::endl;
//                    std::cout << __LINE__ << "eigenvectors:\n" << eigen_vectors << std::endl;
                }


            }

        }

        void compute_v2( const pcl::PointCloud<pcl::PointNormal>::Ptr&  output){

            int input_point_num = m_input_cloud->points.size();
            output->points.resize(input_point_num,pcl::PointNormal(0.0,0.0,0.0,0.0,0.0,0.0));
            output->height = m_input_cloud->height;
            output->width = m_input_cloud->width;
            output->is_dense = true;

            int rt = 0;

            for(int i = 0 ; i < input_point_num;i++){


                auto& query_point = m_input_cloud->at(i);
                rt = m_tree.radiusSearch (query_point, m_query_radius, m_query_indices, m_query_distance);

                std::size_t point_count;
                point_count = m_query_indices.size ();
                m_NormalEst2d.reset();

                m_NormalEst2d.addCenter(m_input_cloud->at(m_query_indices[0]).x,m_input_cloud->at(m_query_indices[0]).y);

                for (const auto &index : m_query_indices)
                {
                    Scalar x = m_input_cloud->at(index).x , y = m_input_cloud->at(index).y ;
                    m_NormalEst2d.addPoints(x,y);
                }

                m_NormalEst2d.compute(output->points[i].normal_x,output->points[i].normal_y,output->points[i].normal_z,output->points[i].curvature);

            }

        }

        void compute_v3( const pcl::PointCloud<pcl::PointNormal>::Ptr&  output){

            int input_point_num = m_input_cloud->points.size();
            output->points.resize(input_point_num,pcl::PointNormal(0.0,0.0,0.0,0.0,0.0,0.0));
            output->height = m_input_cloud->height;
            output->width = m_input_cloud->width;
            output->is_dense = true;

            int rt = 0;
            std::vector<int> query_indices;
            std::vector<float> query_distance;
#pragma omp parallel for \
firstprivate(m_NormalEst2d,query_indices,query_distance)
            for(int i = 0 ; i < input_point_num;i++){



                auto& query_point = m_input_cloud->at(i);
                rt = m_tree.radiusSearch (query_point, m_query_radius, query_indices, query_distance);

                std::size_t point_count;
                point_count = query_indices.size ();
                m_NormalEst2d.reset();

                m_NormalEst2d.addCenter(m_input_cloud->at(query_indices[0]).x,m_input_cloud->at(query_indices[0]).y);

                for (const auto &index : query_indices)
                {
                    Scalar x = m_input_cloud->at(index).x  , y = m_input_cloud->at(index).y ;
                    m_NormalEst2d.addPoints(x,y);
                }

                m_NormalEst2d.compute(output->points[i].normal_x,output->points[i].normal_y,output->points[i].normal_z,output->points[i].curvature);

            }

        }

    };
}




template<typename PointT,typename TreeType>
void test_tree_type(typename pcl::PointCloud<PointT>::Ptr t_input_cloud, TreeType& t_tree){


    if  constexpr(std::is_same<TreeType, pcl::KdTreeFLANN<PointT>>::value){
        t_tree.setInputCloud(t_input_cloud);

    }
    if  constexpr(std::is_same<TreeType,pcl::octree::OctreePointCloudSearch<PointT>>::value){
        t_tree.setInputCloud(t_input_cloud);
        t_tree.addPointsFromInputCloud ();
    }

}




int
pcl_run ( )
{
     int mode = 5;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr  norm_cloud(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr  point_norm_cloud(new pcl::PointCloud<pcl::PointNormal>);

    // Fill in the cloud data
    cloud->width  = 1500;
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

        common::Time  t1 = common::FromUnixNow() ,t2= common::FromUnixNow();

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        pcl::search::KdTree<pcl::PointXYZ> kdtree_2;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);


        test_tree_type<pcl::PointXYZ>(cloud,kdtree);
        test_tree_type<pcl::PointXYZ>(cloud,octree);

        pcl::copyPointCloud(*cloud, *point_norm_cloud);

        {

            t1 = common::FromUnixNow();
//            pcl::createTree(cloud, kdtree);
            pcl::Normal2dEstimation<pcl::PointXYZ, pcl::KdTreeFLANN<pcl::PointXYZ>> N2d_1(kdtree);
            N2d_1.setInputCloud(cloud);
            N2d_1.compute(point_norm_cloud);
            t2= common::FromUnixNow();
            std::cout << "N2d_1 v1 use time " << common::ToMicroSeconds(t2- t1) << " micro second" << std::endl;

        }

        {

            t1 = common::FromUnixNow();
//            pcl::createTree(cloud, kdtree);
            pcl::Normal2dEstimation<pcl::PointXYZ, pcl::KdTreeFLANN<pcl::PointXYZ>> N2d_1(kdtree);
            N2d_1.setInputCloud(cloud);
            N2d_1.compute_v2(point_norm_cloud);
            t2= common::FromUnixNow();
            std::cout << "N2d_1 v2 use time " << common::ToMicroSeconds(t2- t1) << " micro second" << std::endl;


        }

        {
            t1 = common::FromUnixNow();
//            pcl::createTree(cloud, kdtree);
            pcl::Normal2dEstimation<pcl::PointXYZ, pcl::KdTreeFLANN<pcl::PointXYZ>> N2d_1(kdtree);
            N2d_1.setInputCloud(cloud);
            N2d_1.compute_v3(point_norm_cloud);
            t2= common::FromUnixNow();
            std::cout << "N2d_1 v3 use time " << common::ToMicroSeconds(t2- t1) << " micro second" << std::endl;
        }

        {

            t1 = common::FromUnixNow();
//            pcl::createTree(cloud, kdtree_2);
            pcl::Normal2dEstimation<pcl::PointXYZ, pcl::search::KdTree<pcl::PointXYZ>> N2d_2(kdtree_2);
            N2d_2.setInputCloud(cloud);
            N2d_2.compute(point_norm_cloud);
            t2= common::FromUnixNow();
            std::cout << "N2d_2 v1 use time " << common::ToMicroSeconds(t2- t1) << " micro second" << std::endl;

        }

        {

            t1 = common::FromUnixNow();
//            pcl::createTree(cloud, kdtree_2);
            pcl::Normal2dEstimation<pcl::PointXYZ, pcl::search::KdTree<pcl::PointXYZ>> N2d_2(kdtree_2);
            N2d_2.setInputCloud(cloud);
            N2d_2.compute_v2(point_norm_cloud);
            t2= common::FromUnixNow();
            std::cout << "N2d_2 v2 use time " << common::ToMicroSeconds(t2- t1) << " micro second" << std::endl;

        }



        {

            t1 = common::FromUnixNow();
//            pcl::createTree(cloud, kdtree_2);
            pcl::Normal2dEstimation<pcl::PointXYZ, pcl::search::KdTree<pcl::PointXYZ>> N2d_2(kdtree_2);
            N2d_2.setInputCloud(cloud);
            N2d_2.compute_v3(point_norm_cloud);
            t2= common::FromUnixNow();
            std::cout << "N2d_2 v3 use time " << common::ToMicroSeconds(t2- t1) << " micro second" << std::endl;

        }

        std::cout << "check normal\n";
        sample_angle = 0.0;
        for(int i = 0 ; i < cloud->size(); i++){
            auto a1 = sample_angle;

            auto a2 = std::atan2(point_norm_cloud->points[i].normal_y,point_norm_cloud->points[i].normal_x);

            std::cout << i << ": "<< point_norm_cloud->points[i].normal_x  <<", " << point_norm_cloud->points[i].normal_y << ", "<< a1  << ", "<<  a2 << ", " << (a1-a2)/M_PI   << " \n";
            sample_angle += sample_angle_inc;
        }



        {

            t1 = common::FromUnixNow();
//            pcl::createTree(cloud, octree);
            pcl::Normal2dEstimation<pcl::PointXYZ, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>> N2d_3(octree);
            N2d_3.setInputCloud(cloud);
            N2d_3.compute(point_norm_cloud);
            t2= common::FromUnixNow();
            std::cout << "N2d_3 v1 use time " << common::ToMicroSeconds(t2- t1) << " micro second" << std::endl;

        }

        {

            t1 = common::FromUnixNow();
//            pcl::createTree(cloud, octree);
            pcl::Normal2dEstimation<pcl::PointXYZ, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>> N2d_3(octree);
            N2d_3.setInputCloud(cloud);
            N2d_3.compute_v2(point_norm_cloud);
            t2= common::FromUnixNow();
            std::cout << "N2d_3 v2 use time " << common::ToMicroSeconds(t2- t1) << " micro second" << std::endl;

        }

        {

            // crash


            t1 = common::FromUnixNow();
//            pcl::createTree(cloud, octree);
            pcl::Normal2dEstimation<pcl::PointXYZ, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>> N2d_3(octree);
            N2d_3.setInputCloud(cloud);
            N2d_3.compute_v3(point_norm_cloud);
            t2= common::FromUnixNow();
            std::cout << "N2d_3 v3 use time " << common::ToMicroSeconds(t2- t1) << " micro second" << std::endl;

        }


        {
            using namespace matplot;
            vector_2d x(cloud->size(), std::vector<double>(1,0.0));
            vector_2d y(cloud->size(), std::vector<double>(1,0.0));
            vector_2d u(cloud->size(), std::vector<double>(1,0.0));
            vector_2d v(cloud->size(), std::vector<double>(1,0.0));
            std::vector<double> x2(cloud->size());
            std::vector<double> y2(cloud->size());

//            std::cout << "PLOT:\n";

            sample_angle = 0.0;
            for(int i = 0 ; i < cloud->size(); i++){
                x[i][0] =   cloud->points[i].x;
                y[i][0] =   cloud->points[i].y;
                u[i][0] = 0.1;//point_norm_cloud->points[i].normal_x;
                v[i][0] = 0.1;//point_norm_cloud->points[i].normal_y;
                x2[i]  =   cloud->points[i].x;
                y2[i]  =   cloud->points[i].y;
                auto a1 = sample_angle;
                auto a2 = std::atan2(point_norm_cloud->points[i].normal_y,point_norm_cloud->points[i].normal_x);

//                std::cout << i << ": "<< point_norm_cloud->points[i].normal_x  <<", " << point_norm_cloud->points[i].normal_y << ", "<< a1  << ", "<<  a2 << ", " << (a1-a2)/M_PI   << " \n";
                sample_angle += sample_angle_inc;
//                arrow(cloud->points[i].x, cloud->points[i].y, cloud->points[i].x + point_norm_cloud->points[i].normal_x, cloud->points[i].y + point_norm_cloud->points[i].normal_y);
            }
//            std::cout << "PLOT:\n";

//            scatter(x2, y2);

//            quiver(x, y, u, v);
//            colormap(palette::jet());

//            show();
        }




        pcl::PointXYZ searchPoint = cloud->at(0);

        // Neighbors within voxel search

        std::vector<int> pointIdxVec;

        std::vector<int> indices;
        std::vector<float> pointRadiusSquaredDistance;

        float radius = 0.1;

        std::cout << "check radiusSearch" << std::endl;

        int rt = 0;

#if 0

        t1 = common::FromUnixNow();
        octree.setInputCloud (cloud);
        octree.addPointsFromInputCloud ();
        for(int i = 0 ; i < point_num; i++){
            searchPoint = cloud->at(i);
            rt = octree.radiusSearch (searchPoint, radius, indices, pointRadiusSquaredDistance);
        }
        t2= common::FromUnixNow();
        std::cout << "octree use time " << common::ToMicroSeconds(t2- t1) << " micro second" << std::endl;


        t1 = common::FromUnixNow();
        kdtree.setInputCloud (cloud);
        for(int i = 0 ; i < point_num; i++)
        {
            searchPoint = cloud->at(i);
            rt = kdtree.radiusSearch (searchPoint, radius, indices, pointRadiusSquaredDistance);

        }
        t2= common::FromUnixNow();
        std::cout << "kdtree use time " << common::ToMicroSeconds(t2- t1) << " micro second" << std::endl;


        t1 = common::FromUnixNow();
        kdtree_2.setInputCloud (cloud);
        for(int i = 0 ; i < point_num; i++){
            searchPoint = cloud->at(i);
            rt = kdtree_2.radiusSearch (searchPoint, radius, indices, pointRadiusSquaredDistance);

        }
        t2= common::FromUnixNow();
        std::cout << "kdtree_2 use time " << common::ToMicroSeconds(t2- t1) << " micro second" << std::endl;

#endif


        t1 = common::FromUnixNow();

        using Scalar = float;
        auto& input_cloud = *cloud;



        for(int i = 0 ; i < cloud->size(); i++){
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



        if (0)
        {
            for (std::size_t i = 0; i < indices.size (); ++i)
                std::cout << "    "  <<   (*cloud)[ indices[i] ].x
                          << " " << (*cloud)[ indices[i] ].y
                          << " " << (*cloud)[ indices[i] ].z
                          << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
        }



    }
    return (0);

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
int plt_vector_1() {
    using namespace matplot;
    auto [x, y] = meshgrid(iota(0, 0.2, 2), iota(0, 0.2, 2));
    vector_2d u =
            transform(x, y, [](double x, double y) { return cos(x) * 1; });
    vector_2d v =
            transform(x, y, [](double x, double y) { return sin(x) * 1; });

    quiver(x, y, u, v);

    show();
    return 0;
}

struct RansacCircle{
    const std::vector<float>& input_data;
    std::array<float,3> model;




    void compute(){

    }

};

void find_circle(){
    // initialize PointClouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr init_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZ>);


    float x_data[] = { 0.373606, 0.371867, 0.376897, 0.380235, 0.382097, 0.382402, 0.382985, 0.385293, 0.382493, 0.383328, 0.385298, 0.384259, 0.385049, 0.388407, 0.388446, 0.389492, 0.389958, 0.390829, 0.388589, 0.389913, 0.39164, 0.39168, 0.391999, 0.391031, 0.392903, 0.390998, 0.391, 0.393198, 0.391693, 0.387503, 0.391477, 0.394486, 0.393778, 0.393045, 0.39488, 0.394857, 0.394964, 0.389849, 0.38746, 0.39506, 0.393987, 0.395124, 0.395882, 0.394779, 0.395712, 0.393609, 0.3938, 0.39562, 0.39465, 0.3969, 0.393376, 0.397223, 0.394307, 0.390089, 0.392817, 0.39349, 0.393888, 0.399991, 0.394044, 0.399644, 0.400935, 0.39219, 0.401321, 0.397465, 0.397329, 0.401899, 0.398752, 0.401907, 0.402857, 0.39854, 0.398883, 0.404396, 0.404746, 0.399927, 0.402682, 0.401688, 0.397711, 0.40607, 0.399775, 0.406791, 0.399544, 0.408024, 0.407709, 0.407507, 0.407783, 0.407679, 0.403067, 0.400339, 0.409545, 0.401084, 0.408858, 0.41038, 0.410419, 0.411611, 0.403056, 0.402078, 0.411926, 0.406987, 0.405837, 0.403791, 0.413918, 0.414865, 0.413631, 0.407392, 0.407807, 0.407728, 0.409324, 0.419915, 0.419846, 0.421421, 0.4208, 0.412571, 0.423441, 0.413916, 0.413698, 0.424452, 0.425382, 0.41423, 0.425346, 0.427078, 0.428354, 0.428359, 0.429265, 0.432472, 0.424633, 0.435151, 0.425932, 0.43697, 0.439466, 0.439669, 0.443415, 0.446216, 0.454775 };


    float y_data[] = { -0.0260311, -0.0537758, -0.0220142, -0.00255934, -0.0163005, -0.0111581, -0.00343732, 4.3921e-07, -0.0491919, -0.0466783, -0.0399133, -0.0511732, -0.0477649, -0.00610115, 0.00348722, -0.0174917, -0.0131305, 0.00701734, -0.0455501, -0.0324478, -0.0123073, -0.0158289, -0.000879199, -0.0298916, -0.00176291, -0.0396172, -0.0467224, -0.0256234, -0.0441328, -0.0739197, -0.0494546, 0.0115116, -0.0292129, -0.0389337, -0.00974872, -0.0106348, -0.00531766, -0.0644387, -0.0775222, 0.00975406, -0.035459, -0.0248586, -0.00444152, -0.0310694, -0.0151025, -0.0434543, -0.0416874, -0.0320289, -0.0426731, 0.00890836, -0.0541851, 0.0160538, -0.0534121, -0.078959, -0.0640241, -0.0614175, -0.0696553, 0.00269323, -0.0687712, -0.0206442, -0.00719789, -0.0867408, -0.021634, -0.0602119, -0.0611036, -0.00901964, -0.0521928, 0.0252863, 0.00180849, -0.0594601, -0.0640943, 0.00816873, -0.00817488, -0.0633418, -0.0462862, -0.0562488, -0.0823616, -0.0145853, -0.0753313, 0.0210144, -0.0818066, 0.00549441, 0.0183107, 0.0228856, 0.0284133, 0.0320855, -0.0694143, -0.0847814, 0.0193141, -0.0868216, -0.0358727, 0.017508, 0.0202785, 0.00646657, -0.084412, -0.0898746, 0.0110955, -0.0691495, -0.0793063, -0.093116, -0.0204504, 0.0139701, -0.0390992, -0.0872307, -0.0892354, -0.0920986, -0.093425, 0.0302056, 0.0311478, 0.0151377, 0.0350191, -0.0980704, 0.0323701, -0.0964296, -0.0973581, 0.0362823, 0.022932, -0.0994473, 0.028678, 0.0345767, 0.0375843, 0.0424329, 0.038635, 0.0408811, -0.102953, 0.0500194, -0.106309, 0.0462584, 0.0425375, 0.0405663, 0.0479471, 0.0462249, 0.0502082 };



    // populate our PointCloud with points
    init_cloud->width    = 133;
    init_cloud->height   = 1;
    init_cloud->is_dense = true;
    init_cloud->points.resize (init_cloud->width * init_cloud->height);

    for (pcl::index_t i = 0; i < init_cloud->size (); ++i){
        (*init_cloud)[i].x = x_data[i];
        (*init_cloud)[i].y = y_data[i];
    }

    std::vector<int> inliers;
    // created RandomSampleConsensus object and compute the appropriated model
    pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>::Ptr
            model_circle(new pcl::SampleConsensusModelCircle2D<pcl::PointXYZ> (init_cloud));

    model_circle->setRadiusLimits(0.09,0.11);
    model_circle->setModelConstraints([](auto& x){

        if(x(0) < 0.5){
            return false;
        }else{
            return true;
        }
        return true;

    });

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_circle);
    ransac.setDistanceThreshold (.04);

    ransac.computeModel();
    ransac.getInliers(inliers);
    Eigen::VectorXf model_coefficients;

    ransac.getModelCoefficients(model_coefficients);
    std::cout << "model_coefficients " << model_coefficients << std::endl;

    ransac.refineModel();
    ransac.getModelCoefficients(model_coefficients);
    std::cout << "refineModel model_coefficients " << model_coefficients << std::endl;

}

int
main (int argc, char** argv)
{
#ifdef _OPENMP
    std::cout << "use _OPENMP" <<std::endl;

#endif



//    pcl_run( );
//    plt_vector_1();
    find_circle();

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