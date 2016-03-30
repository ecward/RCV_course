#include <smc_tracking/smc_filter.h>
#include <random>
#include <iostream>
#include <eigen3/Eigen/Householder>

using namespace std;

const int smc_filter::NBR_PARTICLES;
constexpr float smc_filter::MAX_VEL;
constexpr float smc_filter::MAX_ACC;
constexpr float smc_filter::MAX_THETA_ACC;

void smc_filter::propagate(float timestep)
{
    const int N = states.cols();

    //default_random_engine generator;
    std::random_device rd;
    std::mt19937 generator(rd());
    normal_distribution<float> distribution(0.0f, 1.0f);

    // just put some simple random sampling on pos, vel & theta
    // probably put some bounds on the velocities
    for (int i = 0; i < N; ++i) {
        float theta = states(2, i);
        states.col(i).head<2>() += states(3, i)*timestep*Eigen::Vector2f(cos(theta), sin(theta));
        //states(2, i) = fmod(theta + 2.0*M_PI, 2.0f*M_PI);

#ifdef USING_CONSTANT_ACCELERATION

        states(2, i) += states(4, i);
        states(3, i) += states(5, i);
        states(3, i) = max(0.0f, states(3, i)); // min velocity
        states(3, i) = min(MAX_VEL, states(3, i)); // max velocity
        states(4, i) += theta_acc_sigma*distribution(generator);
        states(4, i) = max(-MAX_THETA_ACC, states(4, i)); // min theta acceleration
        states(4, i) = min(MAX_THETA_ACC, states(4, i)); // max theta acceleration
        states(5, i) += vel_acc_sigma*distribution(generator);
        states(5, i) = max(-MAX_ACC, states(5, i)); // min acceleration
        states(5, i) = min(MAX_ACC, states(5, i)); // max acceleration

#else

        theta += theta_sigma*distribution(generator);
        states(2, i) = theta;
        states(3, i) += vel_sigma*distribution(generator);
        states(3, i) = max(0.0f, states(3, i)); // min velocity
        states(3, i) = min(MAX_VEL, states(3, i)); // max velocity

#endif

    }
}

void smc_filter::resample(const kdtree& points)
{
    const int N = states.cols();

    const float denom = -2.0f*detect_sigma*detect_sigma;
    const float worst_likelihood = exp(search_radius*search_radius/denom);
    //const float normalization = 1.0f/(detect_sigma*sqrt(2.0f*M_PI));

    // treat the points as a mixture of gaussians with eq weight
    // do we just include the likelihoods in the weights?
    for (int i = 0; i < N; ++i) {
        const float query_pt[2] = {states(0, i), states(1, i)};

        vector<pair<long int, float> > ret_matches;
        nanoflann::SearchParams params;
        //params.sorted = false;

        const size_t nbr_matches = points.index->radiusSearch(&query_pt[0], search_radius, ret_matches, params);

        float obs_likelihood = worst_likelihood;
        for (int j = 0; j < nbr_matches; ++j) {
            obs_likelihood = max(obs_likelihood, exp(ret_matches[j].second*ret_matches[j].second/denom));
        }

        /*
        if (obs_likelihood > 0) {
            weights(i) += log(obs_likelihood); // if we do it like this we do not need normalization, exp
        }
        */
        weights(i) = obs_likelihood;
    }

    float wsum = weights.sum();
    if (wsum > 0) {
        weights /= wsum;
    }
    float wmax = weights.maxCoeff();

    //cout << weights.transpose() << endl;
    //cout << "X: " << states.row(0) << endl;
    //cout << "Max: " << wmax << endl;
    //cout << "Mean: " << weights.mean() << endl;

    //default_random_engine generator;
    std::random_device rd;
    std::mt19937 generator(rd());
    uniform_int_distribution<int> int_dist(0, N-1);

    std::uniform_real_distribution<float> float_dist(0, wmax);

    Eigen::Matrix<float, 6, Eigen::Dynamic> old_states = states;
    Eigen::VectorXf old_weights = weights;

    // now resample based on the weights
    for (int i = 0; i < N; ++i) {
        // 1. randomize index
        // 2. randomize uniform prob
        // 3. if weight < random, go back to 1.
        // 4. take this as new particle
        
        while (true) {
            int j = int_dist(generator);
            if (wmax == 0 || old_weights(j) > float_dist(generator)) {
                weights(i) = 1.0f;
                states.col(i) = old_states.col(j);
                break;
            }
        }

    }

    estimates_updated = false;
}

void smc_filter::compute_estimates()
{
    //mu = (weights.array()*states.array()).rowwise().sum()/weights.sum();
    mu.setZero();
    pos_sigma.setZero();
    const int N = states.cols();
    for (int i = 0; i < N; ++i) {
        mu += weights(i)*states.col(i);
        pos_sigma += weights(i)*states.block<2, 1>(0, i)*states.block<2, 1>(0, i).transpose();
    }
    mu /= weights.sum();
    pos_sigma /= weights.sum();
    estimates_updated = true;
}

Eigen::Vector4f smc_filter::estimate()
{
    if (!estimates_updated) {
        compute_estimates();
    }
    return mu.head<4>();
}

// if something is explained well by one filter, add it to that
// otherwise, initiate its own filter for that observation
// for this, just look at a fitted gaussian, otherwise will be too costly
float smc_filter::likelihood(const Eigen::Vector2f& point)
{
    if (!estimates_updated) {
        compute_estimates();
    }
    Eigen::Vector2f offset = point - mu.head<2>();
    float normalization = 1.0f/sqrt(4.0f*M_PI*M_PI*pos_sigma.determinant());
    cout << mu.transpose() << endl;
    cout << pos_sigma << endl;
    /*
    if (fabs(mu(1)) > 1e6f) {
        cout << "X: " << states.row(0) << endl;
    }
    */
    float likelihood_value = normalization*exp(-0.5f*offset.transpose()*pos_sigma.householderQr().solve(offset));
    cout << "Likelihood: " << likelihood_value << endl;
    cout << "Point: " << point.transpose() << endl;
    /*if (std::isnan(likelihood_value)) {
        cout << "X: " << states.row(0) << endl;
        cout << "Y: " << states.row(1) << endl;
        cout << "Weights: " << weights.transpose() << endl;
    }*/
    return likelihood_value;
}

void smc_filter::init_with_observation(const Eigen::Vector2f& point)
{
    // sample gaussian around point, uniformly sample angle and velocity
    //default_random_engine generator;
    std::random_device rd;
    std::mt19937 generator(rd());
    normal_distribution<float> distribution(0.0f, detect_sigma);
    std::uniform_real_distribution<float> float_dist(0.0f, 1.0f);

    weights.resize(NBR_PARTICLES);
    weights.setOnes();
    states.resize(6, NBR_PARTICLES);
    for (int i = 0; i < NBR_PARTICLES; ++i) {
        Eigen::Matrix<float, 6, 1> new_particle;
        new_particle.head<2>() = point;
        new_particle(0) += distribution(generator);
        new_particle(1) += distribution(generator);
        new_particle(2) = 2.0f*M_PI*float_dist(generator);
        new_particle(3) = MAX_VEL*float_dist(generator); // ~50 km/h max

#ifdef USING_CONSTANT_ACCELERATION

        new_particle(4) = MAX_THETA_ACC*float_dist(generator);
        new_particle(5) = MAX_ACC*float_dist(generator);

#endif

        states.col(i) = new_particle;
    }

    //cout << "X0: " << states.row(0) << endl;
    //cout << "Weights0: " << weights.transpose() << endl;

}

vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >
smc_filter::get_particles()
{
    const int N = states.cols();

    vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > particles(N);
    for (int i = 0; i < N; ++i) {
        particles[i] = states.block<4, 1>(0, i);
    }

    return particles;
}
