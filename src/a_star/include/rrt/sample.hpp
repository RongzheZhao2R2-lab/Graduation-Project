#pragma once

#include <random>

class RandomSampler
{
public:
    RandomSampler()
    {
        std::random_device rd;
        gen_ = std::mt19937_64(rd());
        uniform_rand_ = std::uniform_real_distribution<double>(0.0, 1.0);
        uniform_rand_symmetry = std::uniform_real_distribution<double>(-1.0, 1.0);
    }

    double getRandom()
    {
        return uniform_rand_(gen_);
    }

    double getRandomSymmetry()
    {
        return uniform_rand_symmetry(gen_);
    }
    
    void getPlaneRandom(const double width, const double height, double &res_w, double &res_h)
    {
        res_w = width * getRandomSymmetry();
        res_h = height * getRandomSymmetry();
    }

private:
    std::mt19937_64 gen_;
    std::uniform_real_distribution<double> uniform_rand_;
    std::uniform_real_distribution<double> uniform_rand_symmetry;
};