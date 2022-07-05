#include "math/plane.hpp"

#include <Eigen/Core>
#include <Eigen/SVD>

namespace Cork::Math
{
    BestFitPlaneEquation::BestFitPlaneEquation(uint32_t num_verts, Vector3DVector::const_iterator itr_begin,
                                               Vector3DVector::const_iterator itr_end)
    {
        //  We will use SVD to determine the best-fit plane.  Since we are using a Nx3 matrix,
        //      the normal to the best-fit plane will be the right singular vector associated
        //      with the smallest singular value.

        centroid_ = Math::centroid(itr_begin, itr_end);

        Eigen::MatrixXd A(num_verts, 3);

        int k = 0;
        for (auto itr = itr_begin; itr != itr_end; itr++)
        {
            Vector3D translated_point = *itr - centroid_;

            A(k, 0) = translated_point.x();
            A(k, 1) = translated_point.y();
            A(k, 2) = translated_point.z();

            k++;
        }

        Eigen::BDCSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);

        //  The best-fit normal is the V column whose index is given by the index of the min svd value.
        //      ONly three to check - not worth a loop.

        double min_value = svd.singularValues()[0];
        uint32_t min_index = 0;

        if (svd.singularValues()[1] < min_value)
        {
            min_value = svd.singularValues()[1];
            min_index = 1;
        }

        if (svd.singularValues()[2] < min_value)
        {
            min_value = svd.singularValues()[2];
            min_index = 2;
        }

        //  Pull the values and return them in a Vector3D

        unit_normal_ = Vector3D(svd.matrixV()(0, min_index), svd.matrixV()(1, min_index), svd.matrixV()(2, min_index));

        //  Compute the rms error

        rms_error_ = 0;

        for (auto itr = itr_begin; itr != itr_end; itr++)
        {
            double dist = distance(*itr);

            rms_error_ += dist * dist;
        }

        rms_error_ = sqrt(rms_error_);
    }

}  // namespace Cork::Math
