#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include "camera_pipeline_interface.hpp"
#include "image.hpp"
#include "pixel.hpp"

#include<iostream>
using namespace std;

class CameraPipeline : public CameraPipelineInterface {
 public:
    
  explicit CameraPipeline(CameraSensor* sensor)
    : CameraPipelineInterface(sensor) {}
    
 private:
  using T = typename CameraSensor::T;
  using CameraPipelineInterface::sensor_;

  std::unique_ptr<Image<RgbPixel>> ProcessShot() const override;

  // BEGIN: CS348K STUDENTS MODIFY THIS CODE
  //
  // You can add any necessary private member variables or functions.

  // Handle the edge case of the row/col input (mirror the edge)
  // Return the mirrored index of the image
  //
  inline int miridx(int input, int max_idx) const {
    input = input < 0 ? -input : input;
    input = input >= max_idx ? 2 * (max_idx - 1) - input : input;
    return input;
  }

  // A test tangent mapping for the scaled pixel.
  //
  inline float tan_map(float input) const {
    return 42.36486 * (3.00957 + tan(2.5 * input - 1.25));
  }

  // Check the defects by finding outstanding weird pixel.
  //
  inline float check_defect(float r, float g, float b) const {
    float thre = 0.000000001f;
    bool r_sat = (abs(r - 0.f) < thre) || (abs(r - 1.f) < thre);
    bool g_sat = (abs(g - 0.f) < thre) || (abs(g - 1.f) < thre);
    bool b_sat = (abs(b - 0.f) < thre) || (abs(b - 1.f) < thre);
    return (r_sat && !g_sat && !b_sat) ||
            (!r_sat && g_sat && !b_sat) || (!r_sat && !g_sat && b_sat);
  }

  inline float min4(float a, float b, float c, float d) const {
    return min(min(min(a,b),c),d);
  }

  inline float max4(float a, float b, float c, float d) const {
    return max(max(max(a,b),c),d);
  }  

  //
  // END: CS348K STUDENTS MODIFY THIS CODE  
};
