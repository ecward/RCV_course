#ifndef SMC_FILTER_H
#define SMC_FILTER_H

#include <eigen3/Eigen/Dense>
#include <nanoflann/nanoflann.hpp>

#define USING_CONSTANT_ACCELERATION

class smc_filter {

private:

    static const int NBR_PARTICLES = 1000;
    static constexpr float MAX_VEL = 8.0f;
    static constexpr float MAX_ACC = 0.1f;
    static constexpr float MAX_THETA_ACC = 0.1f;

    float theta_sigma; // only needed if not USING_CONSTANT_ACCELERATION
    float vel_sigma; // only needed if not USING_CONSTANT_ACCELERATION
    float search_radius;
    float detect_sigma;
    float theta_acc_sigma; // only needed if USING_CONSTANT_ACCELERATION
    float vel_acc_sigma; // only needed if USING_CONSTANT_ACCELERATION

    // x, y, theta, vel, theta_acc, vel_acc
    Eigen::Matrix<float, 6, Eigen::Dynamic> states;
    Eigen::VectorXf weights;

    Eigen::Matrix<float, 6, 1> mu;
    Eigen::Matrix2f pos_sigma;
    bool estimates_updated;

    void compute_estimates();

public:

    using kdtree = nanoflann::KDTreeEigenMatrixAdaptor<Eigen::Matrix<float, Eigen::Dynamic, 2>, 2, nanoflann::metric_L2_Simple>;

    void propagate(float timestep);
    void resample(const kdtree& points);
    float likelihood(const Eigen::Vector2f& point);
    void init_with_observation(const Eigen::Vector2f& point);
    Eigen::Vector4f estimate();
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > get_particles();

    smc_filter(float theta_sigma = 0.3f, float vel_sigma = 0.2f, float search_radius = 2.0f,
               float detect_sigma = 0.5f, float theta_acc_sigma = 0.01, float vel_acc_sigma = 0.05) :
               theta_sigma(theta_sigma), vel_sigma(vel_sigma), search_radius(search_radius), detect_sigma(detect_sigma),
               theta_acc_sigma(theta_acc_sigma), vel_acc_sigma(vel_acc_sigma), estimates_updated(false)

    {

    }
};

#endif // SMC_FILTER_H
