//
// Created by waxz on 23-3-10.
//

#include "pcl_icp_with_normal.h"
namespace icp{
    void PclIcpWithNormal::setIcp(){
        te->setWarpFunction(warp_func);
        icp.setTransformationEstimation(te);

        icp.setUseSymmetricObjective(true);
        icp.setUseReciprocalCorrespondences(true);

        setReject(0.5,0.5);


        icp.addCorrespondenceRejector (cor_rej_dist);

//        icp.addCorrespondenceRejector (cor_rej_tri);

//        icp.addCorrespondenceRejector (cor_rej_var);

        icp.addCorrespondenceRejector (cor_rej_norm);
        icp.setCorrespondenceEstimation (ces);

    }

    void PclIcpWithNormal::setReject(float max_dist, float min_normal_cos){
        cor_rej_dist->setMaximumDistance(max_dist);
        cor_rej_norm->setThreshold(min_normal_cos);
    }


    float PclIcpWithNormal::compute(CloudT::Ptr src, CloudT::Ptr tgt,  CloudT::Ptr align, Eigen::Matrix4f& init_pose, Eigen::Matrix4f& final_pose){

        icp.setInputSource(src);
        icp.setInputTarget(tgt);
        icp.align(*align, init_pose);

        final_pose = icp.getFinalTransformation();

        return icp.getFitnessScore();
    }

}