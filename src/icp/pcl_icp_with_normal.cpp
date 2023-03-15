//
// Created by waxz on 23-3-10.
//

#include "pcl_icp_with_normal.h"
namespace icp{
    void PclIcpWithNormal::setIcp(){
        te->setWarpFunction(warp_func);
        icp.setTransformationEstimation(te);

//        icp.setUseSymmetricObjective(true);
//        icp.setUseReciprocalCorrespondences(true);



        icp.setEuclideanFitnessEpsilon(1e-7);
        icp.setTransformationRotationEpsilon(1e-8);
        icp.setTransformationEpsilon(1e-8);





        icp.addCorrespondenceRejector (cor_rej_dist);

//        icp.addCorrespondenceRejector (cor_rej_tri);

//        icp.addCorrespondenceRejector (cor_rej_var);

        icp.addCorrespondenceRejector (cor_rej_norm);
        icp.setCorrespondenceEstimation (ces);

    }

    void PclIcpWithNormal::setReject(int max_iter,int ransac_iter ,float reject_max_dist, float match_max_dist, float ransac_max_dist,  float min_normal_cos){
        icp.setMaximumIterations(max_iter);
        icp.setRANSACIterations(ransac_iter);
        cor_rej_dist->setMaximumDistance(reject_max_dist);
        cor_rej_norm->setThreshold(min_normal_cos);
        icp.setMaxCorrespondenceDistance(match_max_dist);
        icp.setRANSACOutlierRejectionThreshold(ransac_max_dist);

    }


    float PclIcpWithNormal::compute(CloudT::Ptr src, CloudT::Ptr tgt,  CloudT::Ptr align, Eigen::Matrix4f& init_pose, Eigen::Matrix4f& final_pose){

        icp.setInputSource(src);
        icp.setInputTarget(tgt);
        icp.align(*align, init_pose);

        final_pose = icp.getFinalTransformation();

        return icp.getFitnessScore();
    }

}