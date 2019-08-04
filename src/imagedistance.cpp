#include <imagedistance.hpp>

namespace imagedistance
{
    namespace internal
    {
        Histogram CalcHistogram(int num_bins, const Eigen::MatrixXd& channel);
    }
} // namespace imagedistance

imagedistance::Histogram imagedistance::internal::CalcHistogram(const int num_bins, const Eigen::MatrixXd& channel)
{
    Histogram histogram = Histogram::Zero(num_bins);

    const int w = channel.cols();
    const int h = channel.rows();

    // Note: the following double loop is probably able to be implemented using element-wise operations

    for (int x = 0; x < w; ++x)
    {
        for (int y = 0; y < h; ++y)
        {
            const double& value = channel(y, x);

            for (unsigned i = 0; i < num_bins; ++i)
            {
                if (value <= (static_cast<double>(i + 1) / static_cast<double>(num_bins)))
                {
                    histogram(i) += 1.0;
                    break;
                }
            }
        }
    }

    return histogram / static_cast<double>(w * h);
}

imagedistance::ImageDistanceObject::ImageDistanceObject(
    const Eigen::MatrixXd&                                        r_channel,
    const Eigen::MatrixXd&                                        g_channel,
    const Eigen::MatrixXd&                                        b_channel,
    const std::function<Eigen::Vector3d(const Eigen::Vector3d&)>& rgb_to_hsl_converter,
    const int                                                     num_bins)
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
