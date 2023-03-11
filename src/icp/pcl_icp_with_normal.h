//
// Created by waxz on 23-3-10.
//

#ifndef CMAKE_SUPER_BUILD_PCL_ICP_WITH_NORMAL_H
#define CMAKE_SUPER_BUILD_PCL_ICP_WITH_NORMAL_H

#include <pcl/point_types.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid_3d.h>

#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_2D.h>

#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>


namespace icp{


    class PclIcpWithNormal {

    public:
        using PointT = pcl::PointNormal;
        using CloudT = pcl::PointCloud<PointT>;
        using ICPTYPE = pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal>;
        using ICPWarpType = pcl::registration::WarpPointRigid3D<pcl::PointNormal, pcl::PointNormal>;
        using ICPCensType =  pcl::registration::CorrespondenceEstimation<pcl::PointNormal, pcl::PointNormal, float>;

        PclIcpWithNormal():
                te(new ICPTYPE),warp_func(new ICPWarpType),ces(new ICPCensType),
                cor_rej_one(new pcl::registration::CorrespondenceRejectorOneToOne),
                cor_rej_dist(new pcl::registration::CorrespondenceRejectorDistance),
                cor_rej_var(new pcl::registration::CorrespondenceRejectorVarTrimmed),
                cor_rej_norm (new pcl::registration::CorrespondenceRejectorSurfaceNormal),
                cor_rej_tri (new pcl::registration::CorrespondenceRejectorTrimmed)
        {
            setIcp();
        }

        void setIcp();

        float compute(CloudT::Ptr src, CloudT::Ptr tgt,CloudT::Ptr align,  Eigen::Matrix4f& init_pose, Eigen::Matrix4f& final_pose );

        void setReject(float max_dist, float min_normal_cos);
    private:
        pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;

        ICPTYPE::Ptr te;
        ICPWarpType::Ptr warp_func;
        ICPCensType::Ptr ces;


        pcl::registration::CorrespondenceRejectorOneToOne::Ptr cor_rej_one;
        pcl::registration::CorrespondenceRejectorDistance::Ptr cor_rej_dist;
        pcl::registration::CorrespondenceRejectorVarTrimmed::Ptr cor_rej_var;
        pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr cor_rej_norm;
        pcl::registration::CorrespondenceRejectorTrimmed::Ptr cor_rej_tri;

    };

}



#endif //CMAKE_SUPER_BUILD_PCL_ICP_WITH_NORMAL_H
