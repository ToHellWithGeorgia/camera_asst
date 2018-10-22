#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include "camera_pipeline_interface.hpp"
#include "image.hpp"
#include "pixel.hpp"

#include <iostream>
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

  inline float min4(float a, float b, float c, float d) const {
    return min(min(min(a,b),c),d);
  }

  inline float max4(float a, float b, float c, float d) const {
    return max(max(max(a,b),c),d);
  }

  inline float stdev3(float a, float b, float c) const {
    float mean = (a+b+c) / 3.f;
    return sqrt((1.f/3.f) * ((a - mean) * (a - mean) + (b - mean) * (b - mean) +
                             (c - mean) * (c - mean)));
  }

  inline float gausscurve(float i, float sigma = 0.2f) const {
    return exp(-((i - 0.5f) * (i - 0.5f)) / (2 * sigma * sigma));
  }

  inline float coswin(int row, int col, float n) const {
    float PI = 3.14159265;
    float x = sqrt((float)(row*row + col*col));
    return 0.5 - 0.5 * cos(2*PI*(x+0.5f)/n);
  }

  // Convolve the image with the kernel.
  //
  void convolve(std::unique_ptr<Image<RgbPixel>>& image, 
                int width, int height, int kernel_length,
                float* kernel, bool is_abs = false) const;

  // Calculate the gaussian pyramid and save to the vector
  //
  void gp_pyramid(std::unique_ptr<Image<RgbPixel>>& image,
                  std::vector<std::unique_ptr<Image<RgbPixel>>>& out,
                  int width, int height, int depth, bool is_gauss = true) const;

  // Calculate the Laplacian pyramid and save to the vector
  //
  void lp_pyramid(std::unique_ptr<Image<RgbPixel>>& image,
                  std::vector<std::unique_ptr<Image<RgbPixel>>>& out,
                  int width, int height, int depth) const;

  // Reconstruct the image using the laplacian pyramid
  //
  void lp_reconstruct(std::vector<std::unique_ptr<Image<RgbPixel>>>& out,
                      int width, int height) const;

  // Convolve and fuse the pyramid with the weight
  //
  void lp_fuse(std::vector<std::unique_ptr<Image<RgbPixel>>>& out,
               std::vector<std::unique_ptr<Image<RgbPixel>>>& lp1,
               std::vector<std::unique_ptr<Image<RgbPixel>>>& gp1,
               std::vector<std::unique_ptr<Image<RgbPixel>>>& lp2,
               std::vector<std::unique_ptr<Image<RgbPixel>>>& gp2,
               int width, int height) const;

  // Align 2 Gaussian pyramid, by calculating the offset and distance.
  // There is a search radius associated.
  //
  void gp_align(std::vector<std::unique_ptr<Image<RgbPixel>>>& gp1,
                std::vector<std::unique_ptr<Image<RgbPixel>>>& gp2,
                std::unique_ptr<Image<RgbPixel>>& out,
                int width, int height, int depth,
                int tile, int stride, int radi) const;

  //
  // END: CS348K STUDENTS MODIFY THIS CODE  
};
