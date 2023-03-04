#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include "boost/filesystem.hpp"

using namespace std;

void validateArgs(int argc, char *argv[], bool& isCSV);

/**
  * Code example for ICP taking 2 points clouds (2D or 3D) relatively close
  * and computing the transformation between them.
  */
int main(int argc, char *argv[])
{
    bool isCSV = true;

    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;

    char * filename1 = "cloud.pcd";
    char * filename2 = "cloud2.pcd";

    // Load point clouds
    const DP data(DP::load(filename1));
    const DP ref(DP::load(filename2));

    return 0;


    // Create the default ICP algorithm
    PM::ICP icp;
    PointMatcherSupport::Parametrizable::Parameters params;
    std::string name;

    // Prepare reading filters
    name = "MinDistDataPointsFilter";
    params["minDist"] = "1.0";
    std::shared_ptr<PM::DataPointsFilter> minDist_read =
            PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    name = "RandomSamplingDataPointsFilter";
    params["prob"] = "0.05";
    std::shared_ptr<PM::DataPointsFilter> rand_read =
            PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    // Prepare reference filters
    name = "MinDistDataPointsFilter";
    params["minDist"] = "1.0";
    std::shared_ptr<PM::DataPointsFilter> minDist_ref =
            PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    name = "RandomSamplingDataPointsFilter";
    params["prob"] = "0.05";
    std::shared_ptr<PM::DataPointsFilter> rand_ref =
            PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    // Prepare matching function
    name = "KDTreeMatcher";
    params["knn"] = "1";
    params["epsilon"] = "3.16";
    std::shared_ptr<PM::Matcher> kdtree =
            PM::get().MatcherRegistrar.create(name, params);
    params.clear();

    // Prepare outlier filters
    name = "TrimmedDistOutlierFilter";
    params["ratio"] = "0.75";
    std::shared_ptr<PM::OutlierFilter> trim =
            PM::get().OutlierFilterRegistrar.create(name, params);
    params.clear();

    // Prepare error minimization
    name = "PointToPointErrorMinimizer";
    std::shared_ptr<PM::ErrorMinimizer> pointToPoint =
            PM::get().ErrorMinimizerRegistrar.create(name);

    // Prepare transformation checker filters
    name = "CounterTransformationChecker";
    params["maxIterationCount"] = "150";
    std::shared_ptr<PM::TransformationChecker> maxIter =
            PM::get().TransformationCheckerRegistrar.create(name, params);
    params.clear();

    name = "DifferentialTransformationChecker";
    params["minDiffRotErr"] = "0.001";
    params["minDiffTransErr"] = "0.01";
    params["smoothLength"] = "4";
    std::shared_ptr<PM::TransformationChecker> diff =
            PM::get().TransformationCheckerRegistrar.create(name, params);
    params.clear();

    // Prepare inspector
    std::shared_ptr<PM::Inspector> nullInspect =
            PM::get().InspectorRegistrar.create("NullInspector");


// Prepare transformation
    std::shared_ptr<PM::Transformation> rigidTrans =
            PM::get().TransformationRegistrar.create("RigidTransformation");

    // Build ICP solution
    icp.readingDataPointsFilters.push_back(minDist_read);
    icp.readingDataPointsFilters.push_back(rand_read);

    icp.referenceDataPointsFilters.push_back(minDist_ref);
    icp.referenceDataPointsFilters.push_back(rand_ref);

    icp.matcher = kdtree;

    icp.outlierFilters.push_back(trim);

    icp.errorMinimizer = pointToPoint;

    icp.transformationCheckers.push_back(maxIter);
    icp.transformationCheckers.push_back(diff);

    // toggle to write vtk files per iteration
    icp.inspector = nullInspect;
    //icp.inspector = vtkInspect;

    icp.transformations.push_back(rigidTrans);

    // See the implementation of setDefault() to create a custom ICP algorithm
//    icp.setDefault();

    // Compute the transformation to express data in ref
    PM::TransformationParameters T = icp(data, ref);

    // Transform data to express it in ref
    DP data_out(data);
    icp.transformations.apply(data_out, T);

    // Safe files to see the results
    ref.save("test_ref.vtk");
    data.save("test_data_in.vtk");
    data_out.save("test_data_out.vtk");
    cout << "Final transformation:" << endl << T << endl;

    return 0;
}

void validateArgs(int argc, char *argv[], bool& isCSV )
{
    if (argc != 3)
    {
        cerr << "Wrong number of arguments, usage " << argv[0] << " reference.csv reading.csv" << endl;
        cerr << "Will create 3 vtk files for inspection: ./test_ref.vtk, ./test_data_in.vtk and ./test_data_out.vtk" << endl;
        cerr << endl << "2D Example:" << endl;
        cerr << "  " << argv[0] << " ../../examples/data/2D_twoBoxes.csv ../../examples/data/2D_oneBox.csv" << endl;
        cerr << endl << "3D Example:" << endl;
        cerr << "  " << argv[0] << " ../../examples/data/car_cloud400.csv ../../examples/data/car_cloud401.csv" << endl;
        exit(1);
    }
}