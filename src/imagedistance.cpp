#include <imagedistance.hpp>

imagedistance::ImageDistanceObject::ImageDistanceObject(const Eigen::MatrixXd& r_channel,
                                                        const Eigen::MatrixXd& g_channel,
                                                        const Eigen::MatrixXd& b_channel,
                                                        const int              num_bins)
{
    // TODO
}

double imagedistance::CalcL2Distance(const Histogram& a, const Histogram& b)
{
    // TODO
}

double imagedistance::CalcSmoothedL2Distance(const Histogram& a, const Histogram& b)
{
    // TODO
}

double imagedistance::CalcSymmetricKlDivergenceDistance(const Histogram& a, const Histogram& b)
{
    // TODO
}

double imagedistance::CalcEntropyDistance(const Histogram& a, const Histogram& b)
{
    // TODO
}
