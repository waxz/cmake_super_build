//
// Created by waxz on 23-3-10.
//


#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/crop_hull.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid_3d.h>
#include <pcl/registration/transformation_estimation_2D.h>

#include <pcl/registration/joint_icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>

#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_point_to_plane_weighted.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls_weighted.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/joint_icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/gicp6d.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_validation_euclidean.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/pyramid_feature_matching.h>



#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>
#include <pcl/octree/octree_pointcloud_occupancy.h>
#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>

#include <pcl/features/fpfh_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/extract_indices.h>


#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>


#include <pcl/common/transforms.h>

#include "warp_point_rigid_3d_v2.h"


#include "transform/transform.h"


#include "icp/pcl_icp_with_normal.h"

#include "math/random.h"

int main(int argc, char** argv){
    using ICPTYPE = pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal>;
    using ICPWarpType = pcl::registration::WarpPointRigid3D_V2<pcl::PointNormal, pcl::PointNormal>;
    using ICPCensType =  pcl::registration::CorrespondenceEstimation<pcl::PointNormal, pcl::PointNormal, float>;

    // read file

    pcl::PointCloud<pcl::PointNormal>::Ptr  t_reading(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr  t_input(new pcl::PointCloud<pcl::PointNormal>);

    pcl::PointCloud<pcl::PointNormal>::Ptr  t_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr  t_align(new pcl::PointCloud<pcl::PointNormal>);

    if (pcl::io::loadPCDFile<pcl::PointNormal> ("cloud.pcd", *t_reading) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file %s \n","cloud.pcd");
        return 0;
    }


    for(int i = 0; i < 100; i++){
        Eigen::Matrix4f true_transform(Eigen::Matrix4f::Identity ());

        transform::Transform2d est_pose(0.1,0.06,0.14);

        true_transform(0, 0) = est_pose.matrix[0][0];
        true_transform(0, 1) = est_pose.matrix[0][1];
        true_transform(1, 0) = est_pose.matrix[1][0];
        true_transform(1, 1) = est_pose.matrix[1][1];
        true_transform(0, 3) = est_pose.matrix[0][2];
        true_transform(1, 3) = est_pose.matrix[1][2];

        pcl::transformPointCloud( *t_reading, *t_cloud, true_transform );




        {
            int valid_num = 0;
            t_input->points.resize(t_reading->size());

            for(size_t i = 0 ; i < t_reading->size();  i++){
                auto & p = t_reading->at(i);
                bool valid = sqrt(p.x * p.x + p.y*p.y) < 15;

                t_input->points[valid_num] = p;

                t_input->points[valid_num].x += math::normal_real<float>(0.0f,0.005) +  ( ( ((i%100) > 0) && ((i%100) < 10)) ? 1.0: 0.0 );
                t_input->points[valid_num].y += math::normal_real<float>(0.0f,0.005) + ( ( ((i%100) > 0) && ((i%100) < 10)) ? 1.0: 0.0 );

                valid_num += valid;
            }

            t_input->height = 1;
            t_input->width = valid_num;
            t_input->points.resize(valid_num);

        }



        ICPTYPE::Ptr m_pl_te(new ICPTYPE);
        ICPWarpType::Ptr m_warp_fcn_pl(new ICPWarpType);
        ICPCensType::Ptr cens (new ICPCensType);



        pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> m_pl_icp;

        m_pl_te->setWarpFunction(m_warp_fcn_pl);


        m_pl_icp.setTransformationEstimation(m_pl_te);

        Eigen::Matrix4f initial_guess(Eigen::Matrix4f::Identity ());

        // rejector

        pcl::registration::CorrespondenceRejectorOneToOne::Ptr cor_rej_o2o (new pcl::registration::CorrespondenceRejectorOneToOne);

        pcl::registration::CorrespondenceRejectorVarTrimmed::Ptr cor_rej_var (new pcl::registration::CorrespondenceRejectorVarTrimmed);

        pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr cor_rej_norm (new pcl::registration::CorrespondenceRejectorSurfaceNormal);

        pcl::registration::CorrespondenceRejectorTrimmed::Ptr cor_rej_tri (new pcl::registration::CorrespondenceRejectorTrimmed);
        pcl::registration::CorrespondenceRejectorDistance::Ptr cor_rej_dist (new pcl::registration::CorrespondenceRejectorDistance);
        pcl::registration::CorrespondenceRejectorMedianDistance::Ptr cor_rej_median_dist (new pcl::registration::CorrespondenceRejectorMedianDistance);

        cor_rej_dist->setMaximumDistance(0.5);


//    cens->setInputSource (t_input);
//    cens->setInputTarget (t_cloud);

//    cor_rej_norm->setInputSource<pcl::PointNormal>(t_input);
//    cor_rej_norm->setInputTarget<pcl::PointNormal>(t_cloud);
//    cor_rej_norm->setInputNormals<pcl::PointNormal, pcl::PointNormal>(t_input);
//    cor_rej_norm->setTargetNormals<pcl::PointNormal, pcl::PointNormal>(t_cloud);



        // two point normal vector cross product, aka cos (angle), should be lager than thresh
        cor_rej_norm->setThreshold(0.5);

        cor_rej_median_dist->setMedianFactor(0.8);


        cor_rej_tri->setOverlapRatio(0.5);

//    m_pl_icp.addCorrespondenceRejector (cor_rej_o2o);
//    m_pl_icp.addCorrespondenceRejector (cor_rej_var);
        m_pl_icp.addCorrespondenceRejector (cor_rej_norm);
//    m_pl_icp.addCorrespondenceRejector (cor_rej_tri);
        m_pl_icp.addCorrespondenceRejector (cor_rej_dist);
//    m_pl_icp.addCorrespondenceRejector (cor_rej_median_dist);



        m_pl_icp.setCorrespondenceEstimation (cens);

        m_pl_icp.setInputSource(t_input);
        m_pl_icp.setInputTarget(t_cloud);
        m_pl_icp.setUseSymmetricObjective(true);
        m_pl_icp.setUseReciprocalCorrespondences(true);

        m_pl_icp.align(*t_align, initial_guess);

        icp::PclIcpWithNormal pl_icp;
        Eigen::Matrix4f final_pose(Eigen::Matrix4f::Identity ());



        Eigen::Matrix4f result_pose = m_pl_icp.getFinalTransformation();
        std::cout << "true_transform:\n" << true_transform << std::endl;

        float score_1 = m_pl_icp.getFitnessScore();
        std::cout << "getFinalTransformation:\n" << result_pose << "\n score: " << score_1<< std::endl;

        pl_icp.setReject(50, 1, 0.3, 0.3, 0.3, 0.8);

        float score_2 = pl_icp.compute(t_input, t_cloud, t_align, initial_guess, final_pose);
        std::cout << "final_pose:\n" << final_pose << "\n score: " << score_2 << std::endl;

        float error_1 = (true_transform.inverse()*result_pose)(0,3);
        float error_2 = (true_transform.inverse()*final_pose)(0,3);

        float error_3 = (true_transform.inverse()*result_pose)(1,3);
        float error_4 = (true_transform.inverse()*final_pose)(1,3);

        printf("error [%.3f, %.3f] vs [%.3f, %.3f]\n", error_1,error_3, error_2, error_4);
        if(
//                abs(error_1) > 0.01 || abs(error_2) > 0.01 ||

                 abs(error_2) > 0.01 || abs(error_4) > 0.01

                 ){
            std::cout << "fail at " << i << std::endl;
            break;
        }


    }
    pcl::io::savePCDFileASCII("t_input.pcd", *t_input);



}