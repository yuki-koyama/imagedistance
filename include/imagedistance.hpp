#ifndef IMAGEDISTANCE_HPP
#define IMAGEDISTANCE_HPP

#include <array>
#include <Eigen/Core>

namespace imagedistance
{
    using Histogram = Eigen::VectorXd;

    class ImageDistanceObject
    {
    public:
        ImageDistanceObject(const Eigen::MatrixXd& r_channel,
                            const Eigen::MatrixXd& g_channel,
                            const Eigen::MatrixXd& b_channel,
                            const int              num_bins = 30);

    private:
        std::array<Histogram, 3> m_rgb_histograms;
        std::array<Histogram, 3> m_hsl_histograms;
        Histogram                m_intensity_histogram;
        std::array<Histogram, 2> m_edge_histograms;
    };

    double CalcL2Distance                   (const Histogram& a, const Histogram& b);
    double CalcSmoothedL2Distance           (const Histogram& a, const Histogram& b);
    double CalcSymmetricKlDivergenceDistance(const Histogram& a, const Histogram& b);
    double CalcEntropyDistance              (const Histogram& a, const Histogram& b);
}

#endif /* IMAGEDISTANCE_HPP */
