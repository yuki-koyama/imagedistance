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
    const int w = r_channel.cols();
    const int h = r_channel.rows();

    assert(w == g_channel.cols() && h == g_channel.rows());
    assert(w == b_channel.cols() && h == b_channel.rows());

    // RGB
    {
        m_rgb_histograms[0] = internal::CalcHistogram(num_bins, r_channel);
        m_rgb_histograms[1] = internal::CalcHistogram(num_bins, g_channel);
        m_rgb_histograms[2] = internal::CalcHistogram(num_bins, b_channel);
    }

    // HSL
    {
        Eigen::MatrixXd h_channel(h, w);
        Eigen::MatrixXd s_channel(h, w);
        Eigen::MatrixXd l_channel(h, w);
        for (int x = 0; x < w; ++x)
        {
            for (int y = 0; y < h; ++y)
            {
                const Eigen::Vector3d hsl =
                    rgb_to_hsl_converter(Eigen::Vector3d(r_channel(y, x), g_channel(y, x), b_channel(y, x)));

                h_channel(y, x) = hsl(0);
                s_channel(y, x) = hsl(1);
                l_channel(y, x) = hsl(2);
            }
        }
        m_hsl_histograms[0] = internal::CalcHistogram(num_bins, h_channel);
        m_hsl_histograms[1] = internal::CalcHistogram(num_bins, s_channel);
        m_hsl_histograms[2] = internal::CalcHistogram(num_bins, l_channel);
    }

    // Intensity
    {
        Eigen::MatrixXd i_channel(h, w);
        for (int x = 0; x < w; ++x)
        {
            for (int y = 0; y < h; ++y)
            {
                i_channel(y, x) = (r_channel(y, x) + g_channel(y, x) + b_channel(y, x)) / 3.0;
            }
        }
        m_intensity_histogram = internal::CalcHistogram(num_bins, i_channel);
    }

    // Edge
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
