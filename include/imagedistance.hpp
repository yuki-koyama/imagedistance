#ifndef IMAGEDISTANCE_HPP
#define IMAGEDISTANCE_HPP

#include <Eigen/Core>
#include <array>
#include <functional>

namespace imagedistance
{
    using Histogram = Eigen::VectorXd;

    class HistogramManager
    {
    public:
        HistogramManager(const Eigen::MatrixXd&                                        r_channel,
                         const Eigen::MatrixXd&                                        g_channel,
                         const Eigen::MatrixXd&                                        b_channel,
                         const std::function<Eigen::Vector3d(const Eigen::Vector3d&)>& rgb_to_hsl_converter,
                         const int                                                     num_bins = 30);

        std::array<Histogram, 3> m_rgb_histograms;
        std::array<Histogram, 3> m_hsl_histograms;
        Histogram                m_intensity_histogram;
        std::array<Histogram, 2> m_edge_histograms;

        double m_size;
        double m_aspect;
    };

    // This function returns a 38-dimensional vector, calculated based on Kapoor et al.'s paper [2014]
    Eigen::VectorXd CalcDistances(const HistogramManager& a, const HistogramManager& b);

    double CalcL2Distance(const Histogram& a, const Histogram& b);
    double CalcSmoothedL2Distance(const Histogram& a, const Histogram& b);
    double CalcSymmetricKlDivergenceDistance(const Histogram& a, const Histogram& b);
    double CalcEntropyDistance(const Histogram& a, const Histogram& b);
} // namespace imagedistance

#endif /* IMAGEDISTANCE_HPP */
