#ifndef UTILITY_H
#define UTILITY_H

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <math.h>

#define PI 3.14159f

class Utility {
  public:
    
    Utility();
    ~Utility() {}
    
    const float euclideanDistance(const std::vector<float> a, const std::vector<float> b) const;

    const float findAngleFromAToB(const std::vector<float> a, const std::vector<float> b) const;
    
    const float findDistanceBetweenAngles(const float a1, const float a2) const;
    
    const float displaceAngle(const float a1, float a2) const;
    
    const float getEuclideanDist(const std::vector<float> a, std::vector<float> b) const;
    
};
#endif
