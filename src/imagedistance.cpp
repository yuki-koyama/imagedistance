#include <cmath>
#include <functional>
#include <imagedistance.hpp>
#include <vector>

namespace imagedistance
{
    namespace internal
    {
        Histogram       CalcHistogram(int num_bins, const Eigen::MatrixXd& channel);
        Eigen::MatrixXd ApplySobelFilter(const Eigen::MatrixXd& channel, const bool is_x_direction);
        double          CalcKlDivergence(const Histogram& histogram_a, const Histogram& histogram_b);
        double          CalcEntropy(const Histogram& histogram);
    } // namespace internal
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

Eigen::MatrixXd imagedistance::internal::ApplySobelFilter(const Eigen::MatrixXd& channel, const bool is_x_direction)
{
    const int w = channel.cols();
    const int h = channel.rows();

    const Eigen::Matrix3d kernel = [&is_x_direction]() {
        Eigen::Matrix3d kernel;
        if (is_x_direction) { kernel << -1.0, 0.0, +1.0, -2.0, 0.0, +2.0, -1.0, 0.0, +1.0; }
        else
        {
            kernel << -1.0, -2.0, -1.0, 0.0, 0.0, 0.0, +1.0, +2.0, +1.0;
        }
        return kernel;
    }();

    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(h, w);

    for (int x = 1; x < w - 1; ++x)
    {
        for (int y = 1; y < h - 1; ++y)
        {
            for (int x_k : {-1, 0, 1})
            {
                for (int y_k : {-1, 0, 1})
                {
                    result(y, x) += kernel(y_k + 1, x_k + 1) * channel(y + y_k, x + x_k);
                }
            }
        }
    }

    return result;
}

double imagedistance::internal::CalcKlDivergence(const Histogram& histogram_a, const Histogram& histogram_b)
{
    const int num_bins = histogram_a.size();

    assert(num_bins == histogram_b.size());

    double sum = 0.0;
    for (int i = 0; i < num_bins; ++i)
    {
        sum += histogram_a(i) * (std::log(std::max(histogram_a(i), 1e-32)) - std::log(std::max(histogram_b(i), 1e-32)));
    }

    return sum;
}

double imagedistance::internal::CalcEntropy(const Histogram& histogram)
{
    const int num_bins = histogram.size();

    double sum = 0.0;
    for (int i = 0; i < num_bins; ++i)
    {
        sum += -histogram(i) * std::log(std::max(histogram(i), 1e-32));
    }

    return sum;
}

imagedistance::HistogramManager::HistogramManager(
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
    m_rgb_histograms[0] = internal::CalcHistogram(num_bins, r_channel);
    m_rgb_histograms[1] = internal::CalcHistogram(num_bins, g_channel);
    m_rgb_histograms[2] = internal::CalcHistogram(num_bins, b_channel);

    // HSL
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

    // Intensity
    const Eigen::MatrixXd i_channel = (r_channel + g_channel + b_channel) / 3.0;

    m_intensity_histogram = internal::CalcHistogram(num_bins, i_channel);

    // Edge
    const Eigen::MatrixXd edge_x_channel = internal::ApplySobelFilter(i_channel, true);
    const Eigen::MatrixXd edge_y_channel = internal::ApplySobelFilter(i_channel, false);

    m_edge_histograms[0] = internal::CalcHistogram(num_bins, edge_x_channel);
    m_edge_histograms[1] = internal::CalcHistogram(num_bins, edge_y_channel);

    // Misc.
    m_size   = static_cast<double>(w * h);
    m_aspect = static_cast<double>(h) / static_cast<double>(w);
}

Eigen::VectorXd imagedistance::CalcDistances(const HistogramManager& a, const HistogramManager& b)
{
    Eigen::VectorXd d(38);
    int             current_index = 0;

    const std::vector<std::function<double(const imagedistance::Histogram&, const imagedistance::Histogram&)>> metrics =
        {
            imagedistance::CalcL2Distance,
            imagedistance::CalcSmoothedL2Distance,
            imagedistance::CalcSymmetricKlDivergenceDistance,
            imagedistance::CalcEntropyDistance,
        };

    // Histogram-based distances
    for (const auto& metric : metrics)
    {
        for (unsigned k = 0; k < 3; ++k)
        {
            d(current_index++) = metric(a.m_rgb_histograms[k], b.m_rgb_histograms[k]);
        }
        for (unsigned k = 0; k < 3; ++k)
        {
            d(current_index++) = metric(a.m_hsl_histograms[k], b.m_hsl_histograms[k]);
        }
        d(current_index++) = metric(a.m_intensity_histogram, b.m_intensity_histogram);
        for (unsigned k = 0; k < 2; ++k)
        {
            d(current_index++) = metric(a.m_edge_histograms[k], b.m_edge_histograms[k]);
        }
    }

    // Other distances
    d(current_index++) = std::abs(a.m_aspect - b.m_aspect);
    d(current_index++) = std::abs(a.m_size - b.m_size);

    assert(current_index == 38);

    return d;
}

double imagedistance::CalcL2Distance(const Histogram& a, const Histogram& b) { return (a - b).norm(); }

double imagedistance::CalcSmoothedL2Distance(const Histogram& a, const Histogram& b)
{
    const int num_bins = a.size();

    assert(num_bins == b.size());

    const auto smooth_histogram = [num_bins](const Eigen::VectorXd& v) {
        Eigen::VectorXd result(num_bins);
        result(0) = (v(0) + v(1)) / 2.0;
        for (unsigned elem = 1; elem < num_bins - 1; ++elem)
        {
            result(elem) = (v(elem - 1) + v(elem) + v(elem + 1)) / 3.0;
        }
        result(num_bins - 1) = (v(num_bins - 1) + v(num_bins - 2)) / 2.0;

        return result;
    };

    Eigen::VectorXd a_smoothed = a;
    Eigen::VectorXd b_smoothed = b;

    for (int i = 0; i < 5; ++i)
    {
        a_smoothed = smooth_histogram(a_smoothed);
        b_smoothed = smooth_histogram(b_smoothed);
    }

    return (a_smoothed - b_smoothed).norm();
}

double imagedistance::CalcSymmetricKlDivergenceDistance(const Histogram& a, const Histogram& b)
{
    return internal::CalcKlDivergence(a, b) + internal::CalcKlDivergence(b, a);
}

double imagedistance::CalcEntropyDistance(const Histogram& a, const Histogram& b)
{
    return std::abs(internal::CalcEntropy(a) - internal::CalcEntropy(b));
}
