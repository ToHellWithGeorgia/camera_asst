#include "camera_pipeline.hpp"
#include <chrono>

std::unique_ptr<Image<RgbPixel>> CameraPipeline::ProcessShot() const {
    
  // BEGIN: CS348K STUDENTS MODIFY THIS CODE

  // put the lens cap on if you'd like to measure a "dark frame"
  sensor_->SetLensCap(false);
    
  // grab RAW pixel data from sensor
  const int width = sensor_->GetSensorWidth();
  const int height = sensor_->GetSensorHeight();
  auto raw_data = sensor_->GetSensorData(0, 0, width, height);
  auto burst_data = sensor_->GetBurstSensorData(0, 0, width, height);

  bool is_part1 = true;
  bool is_part2 = true;

  auto t1 = chrono::steady_clock::now();
  // Step 0:
  // Part 2: Perform aligning and merging to the burst photographs
  //
  int burst_size = burst_data.size();
  cout << "Preprocessing burst alignment and merging for " << burst_size 
       << " images...";
  std::vector<std::unique_ptr<Image<RgbPixel>>> burst_gray;

  // Downsampling the Bayer RAW images and transfer into grayscale
  //
  for (int n = 0; n < burst_size; ++n) {
    std::unique_ptr<Image<RgbPixel>> tmpimg(new Image<RgbPixel>(width/2, height/2)); 
    for (size_t row = 0; row < height/2; ++row) {
      for (size_t col = 0; col < width/2; ++col) {
        auto& pixel = (*tmpimg)(row, col);
        pixel.r = pixel.g = pixel.b = (burst_data[n]->data(2*row, 2*col) +
                                       burst_data[n]->data(2*row+1, 2*col) +
                                       burst_data[n]->data(2*row, 2*col+1) +
                                       burst_data[n]->data(2*row+1, 2*col+1)) / 4.f;
      }
    }
    burst_gray.push_back(std::move(tmpimg));
  }

  // Constructing GP for each of the burst image.
  //
  int align_depth = 5;
  std::vector<std::vector<std::unique_ptr<Image<RgbPixel>>>> GP_burst(burst_size);
  for (int n = 0; n < burst_size; ++n) {
    gp_pyramid(burst_gray[n], GP_burst[n], width/2, height/2, align_depth);
  }

  auto t2 = chrono::steady_clock::now();
  cout << " Runtime: " 
       << chrono::duration_cast<chrono::milliseconds>(t2 - t1).count()
       << " ms" << endl;
  // Find the matching tile from the bottom up.
  // We save the offset in r(row),g(col) and distance information in b channel.
  //
  int tile = 16, stride = 8, search_radius = 8;
  cout << "Aligning with tile: " << tile << " stride: " << stride
       << " search radius: " << search_radius;
  std::vector<std::unique_ptr<Image<RgbPixel>>> align_offset;
  for (int n = 0; n < burst_size - 1; ++n) {
    std::unique_ptr<Image<RgbPixel>> offset(new Image<RgbPixel>(width/2, height/2));
    gp_align(GP_burst[0], GP_burst[n+1], offset, width/2, height/2,
             align_depth, tile, stride, search_radius);
    align_offset.push_back(std::move(offset));
  }

  auto t3 = chrono::steady_clock::now();
  cout << " Runtime: " 
       << chrono::duration_cast<chrono::milliseconds>(t3 - t2).count()
       << " ms" << endl;
  // Merge the picture
  //
  cout << "Merging pictures...";
  auto wgt = [](float a) {
    float low = 0.5f;
    float high = 3.0f;
    if (a > high) return 0.0f;
    if (a < low) return 1.0f;
    return (high - a) / (high - low);
  };
    
  // merge the raw frames.
  auto merge_data = burst_data[0]->Clone();
  for (int num = 1; num < burst_size; ++num) {
    for (int row = 0; row < height / 2 - tile; row += stride) {
      for (int col = 0; col < width / 2 - tile; col += stride) {
        float diff = (*align_offset[num-1])(row/stride,col/stride).b;
        int offset_row = (int)round((*align_offset[num-1])(row/stride,col/stride).r);
        int offset_col = (int)round((*align_offset[num-1])(row/stride,col/stride).g);
        float wgt_merge = wgt(diff);

        for (int tile_row = 0; tile_row < tile; ++tile_row) {
          for (int tile_col = 0; tile_col < tile; ++tile_col) {
            float cos_wgt = wgt_merge * coswin(tile_row, tile_col, tile*sqrt(2));
            cos_wgt = cos_wgt / (1.f + cos_wgt);
            for (int raw_row = 0; raw_row < 2; ++raw_row) {
              for (int raw_col = 0; raw_col < 2; ++raw_col) {
                merge_data->data(2*(row+tile_row)+raw_row,2*(col+tile_col)+raw_col) =
                  cos_wgt * burst_data[num]->data(2*(row+tile_row+offset_row)+raw_row,
                                                  2*(col+tile_col+offset_col)+raw_col) +
                  (1-cos_wgt) * 
                  merge_data->data(2*(row+tile_row)+raw_row,2*(col+tile_col)+raw_col);

              }
            }
          }
        }
      }
    }
  }
  raw_data = std::move(merge_data);


  // In this function you should implement your full RAW image processing pipeline.
  //   (1) Demosaicing
  //   (2) Address sensing defects such as bad pixels and image noise.
  //   (3) Apply local tone mapping based on the local laplacian filter or exposure fusion.
  //   (4) gamma correction

  // Pixel is:
  //  G(00) R(01) G R
  //  B(10) G(11) B G
  //  G     R
  //  B     G

  auto t4 = chrono::steady_clock::now();
  cout << " Runtime: " 
       << chrono::duration_cast<chrono::milliseconds>(t4 - t3).count()
       << " ms" << endl;
  // Step 1: detect and correct static defects
  //
  // Calculate the bright difference on the 3x3 tile.
  //
  cout << "Detecting and correcting static defects...";
  auto copy_data = raw_data->Clone();
  for (size_t row = 0; row < height; ++row) {
    for (size_t col = 0; col < width; ++col) {
      copy_data->data(row, col) = 0;
      for (int tilex = -1; tilex <= 1; ++tilex) {
        for (int tiley = -1; tiley <=1; ++tiley) {
          copy_data->data(row, col) += raw_data->data(miridx(row + tilex, height),
                                                      miridx(col + tiley, width));
        }
      }
      copy_data->data(row, col) = raw_data->data(row, col) - 
                                    copy_data->data(row, col) / 9.f;
    }
  }

  std::unique_ptr<Image<RgbPixel>> image(new Image<RgbPixel>(width, height));

  // We detect and correct sensor defects on the Bayer raw image.
  // https://www.ingentaconnect.com/contentone/ist/ei/2017/00002017/00000015/
  // art00008?crawler=true&mimetype=application/pdf
  //
  size_t cnt1 = 0, cnt2 = 0; // Calculate the defection count.
  float M1 = 0.4, M2 = 10.0; // pixel difference threshold
  for (size_t row = 2; row < height - 2; ++row) {
    for (size_t col = 2; col < width - 2; ++col) {
      float cur_pixel = raw_data->data(row, col);
      std::vector<float> tile{raw_data->data(row-2, col-2),
                              raw_data->data(row-2, col),
                              raw_data->data(row-2, col+2),
                              raw_data->data(row, col-2),
                              raw_data->data(row, col),
                              raw_data->data(row, col+2),
                              raw_data->data(row+2, col-2),
                              raw_data->data(row+2, col),
                              raw_data->data(row+2, col+2)};
      std::sort(tile.begin(), tile.end());
      float avg = 0.0f;
      float median = tile[4];
      for (auto it = tile.begin()+1; it != tile.end()-1; ++it) {
        avg += *it;
      }
      avg /= 7.f;

      // Check all the condition
      bool cond1 = cur_pixel > (1.f + M1) * avg;
      bool cond2 = cur_pixel < (1.f - M1) * avg;
      bool cond3 = copy_data->data(row, col) > 
                     M2 * min(copy_data->data(row, col-1), copy_data->data(row, col+1));
      bool cond4 = copy_data->data(row, col) > 
                     M2 * min(copy_data->data(row-1, col), copy_data->data(row+1, col));
      bool cond5 = copy_data->data(row, col) > 
                     M2 * min4(copy_data->data(row-1, col-1),
                               copy_data->data(row-1, col+1),
                               copy_data->data(row+1, col-1),
                               copy_data->data(row+1, col+1));
      bool cond6 = copy_data->data(row, col) < 
                     M2 * max(copy_data->data(row, col-1), copy_data->data(row, col+1));
      bool cond7 = copy_data->data(row, col) < 
                     M2 * max(copy_data->data(row-1, col), copy_data->data(row+1, col));
      bool cond8 = copy_data->data(row, col) <
                     M2 * max4(copy_data->data(row-1, col-1),
                               copy_data->data(row-1, col+1),
                               copy_data->data(row+1, col-1),
                               copy_data->data(row+1, col+1));
      bool is_hot = false, is_cold = false;
      if (cond1 && cond3 && cond4 && cond5) {
        is_hot = true;
        cnt1++;
      } else if (cond2 && cond6 && cond7 && cond8) {
        is_cold = true;
        cnt2++;
      }

      // Do a simple neighbour median for now. 
      //
      if (is_hot || is_cold) {
        raw_data->data(row, col) = median;
      }

    }
  }

  auto t5 = chrono::steady_clock::now();
  cout << " Runtime: " 
       << chrono::duration_cast<chrono::milliseconds>(t5 - t4).count()
       << " ms" << endl;
  // Step 2: 
  // Part 1: We first perform demosaicing by 2x2 tile bilinear
  //
  cout << "Demosaicing...";
  for (size_t row = 0; row < height; row = row + 2) {
    for (size_t col = 0; col < width; col = col + 2) {
      auto& pixel00 = (*image)(row, col);
      auto& pixel01 = (*image)(row, col + 1);
      auto& pixel10 = (*image)(row + 1, col);
      auto& pixel11 = (*image)(row + 1, col + 1);

      int temprow = row;
      int tempcol = col;
      
      pixel00.r = (raw_data->data(temprow, miridx(tempcol - 1, width)) + 
                     raw_data->data(temprow, miridx(tempcol + 1, width))) / 2.f;
      pixel00.g = raw_data->data(temprow, tempcol);
      pixel00.b = (raw_data->data(miridx(temprow - 1, height), tempcol) + 
                     raw_data->data(miridx(temprow + 1, height), tempcol)) / 2.f;

      temprow = row;
      tempcol = col + 1;

      pixel01.r = raw_data->data(temprow, tempcol);
      pixel01.g = (raw_data->data(temprow, miridx(tempcol - 1, width)) + 
                     raw_data->data(temprow, miridx(tempcol + 1, width)) +
                     raw_data->data(miridx(temprow - 1, height), tempcol) + 
                     raw_data->data(miridx(temprow + 1, height), tempcol)) / 4.f;
      pixel01.b = (raw_data->data(miridx(temprow - 1, height), miridx(tempcol - 1, width)) + 
                     raw_data->data(miridx(temprow - 1, height), miridx(tempcol + 1, width)) +
                     raw_data->data(miridx(temprow + 1, height), miridx(tempcol - 1, width)) + 
                     raw_data->data(miridx(temprow + 1, height), miridx(tempcol + 1, width))) / 4.f;

      temprow = row + 1;
      tempcol = col;

      pixel10.r = (raw_data->data(miridx(temprow - 1, height), miridx(tempcol - 1, width)) + 
                     raw_data->data(miridx(temprow - 1, height), miridx(tempcol + 1, width)) +
                     raw_data->data(miridx(temprow + 1, height), miridx(tempcol - 1, width)) + 
                     raw_data->data(miridx(temprow + 1, height), miridx(tempcol + 1, width))) / 4.f;
      pixel10.g = (raw_data->data(temprow, miridx(tempcol - 1, width)) + 
                     raw_data->data(temprow, miridx(tempcol + 1, width)) +
                     raw_data->data(miridx(temprow - 1, height), tempcol) + 
                     raw_data->data(miridx(temprow + 1, height), tempcol)) / 4.f;
      pixel10.b = raw_data->data(temprow, tempcol);

      temprow = row + 1;
      tempcol = col + 1;

      pixel11.r = (raw_data->data(miridx(temprow - 1, height), tempcol) + 
                     raw_data->data(miridx(temprow + 1, height), tempcol)) / 2.f;
      pixel11.g = raw_data->data(temprow, tempcol);
      pixel11.b = (raw_data->data(temprow, miridx(tempcol - 1, width)) + 
                     raw_data->data(temprow, miridx(tempcol + 1, width))) / 2.f;
    }
  }
  
  auto t6 = chrono::steady_clock::now();
  cout << " Runtime: " 
       << chrono::duration_cast<chrono::milliseconds>(t6 - t5).count()
       << " ms" << endl;
  // Step 3:
  // Part 1: Perform a Gaussian blur to the image by defining the Gaussian
  // kernel.
  cout << "Performing Gaussian blur...";
  float gauss_kernel[5] = {0.06136, 0.24477, 0.38774, 0.24477, 0.06136};
  convolve(image, width, height, 5, gauss_kernel);
  
  // Step 4:
  // Part 1: Perform a global tone mapping.
  //
  // Rescale the [0~1] floating number to [0~255]
  // Use a tangent mapping derived by myself.
  if (!is_part2) {
    for (size_t row = 0; row < height; ++row) {
      for (size_t col = 0; col < width; ++col) {
        auto& pixel = (*image)(row, col);
        // pixel.r *= 255.f;
        // pixel.g *= 255.f;
        // pixel.b *= 255.f;
        pixel = pixel.pow(pixel, 0.44) * 255.f;
      }
    }
    return image;
  }


  float bright_scale = 1.3f;
  std::unique_ptr<Image<RgbPixel>> graybrig = image->Clone();
  std::unique_ptr<Image<RgbPixel>> rgbbrig(new Image<RgbPixel>(width, height));
  std::unique_ptr<Image<RgbPixel>> rgbbrig_luma(new Image<RgbPixel>(width, height));
  for (size_t row = 0; row < height; ++row) {
    for (size_t col = 0; col < width; ++col) {
      auto& pixel = (*graybrig)(row, col);
      pixel = pixel.pow(pixel, 0.44f);
      pixel = pixel.RgbToYuv(pixel);
      pixel.y *= bright_scale;
      pixel = pixel.YuvToRgb(pixel);
      (*rgbbrig_luma)(row,col) = pixel;
      (*rgbbrig_luma)(row,col).r = std::min((*rgbbrig_luma)(row,col).r, 1.f);
      (*rgbbrig_luma)(row,col).g = std::min((*rgbbrig_luma)(row,col).g, 1.f);
      (*rgbbrig_luma)(row,col).b = std::min((*rgbbrig_luma)(row,col).b, 1.f);
      pixel = pixel.pow(pixel, 1.f/0.44f);
      (*rgbbrig)(row,col) = pixel;
      (*rgbbrig)(row,col).r = std::min((*rgbbrig)(row,col).r, 1.f);
      (*rgbbrig)(row,col).g = std::min((*rgbbrig)(row,col).g, 1.f);
      (*rgbbrig)(row,col).b = std::min((*rgbbrig)(row,col).b, 1.f);
      pixel = pixel.RgbToYuv(pixel);
      pixel.u = pixel.v = pixel.y;
    }
  }

  // Step 5:
  // Part 2: convert the image to YUV, create a dark and bright grayscale.
  //
  std::unique_ptr<Image<RgbPixel>> graydark(new Image<RgbPixel>(width, height)); 
  std::unique_ptr<Image<RgbPixel>> rgbdark = image->Clone();
  std::unique_ptr<Image<RgbPixel>> rgbdark_luma = image->Clone();
  for (size_t row = 0; row < height; ++row) {
    for (size_t col = 0; col < width; ++col) {
      auto& pixel = (*image)(row, col);
      (*graydark)(row,col) = pixel.RgbToYuv(pixel);

      // Convert all channel to y for visualization.
      (*graydark)(row,col).u = (*graydark)(row,col).v = (*graydark)(row,col).y;
      (*rgbdark_luma)(row,col) = pixel.pow((*rgbdark)(row,col), 0.44f);
    }
  }

  auto t7 = chrono::steady_clock::now();
  cout << " Runtime: " 
       << chrono::duration_cast<chrono::milliseconds>(t7 - t6).count()
       << " ms" << endl;
  // Step 6:
  // Part 2: Calculate the weight of the contrast, saturation and 
  // well-exposedness.
  //
  cout << "Calculating weight map...";
  // Contrast - laplacian filter
  //
  std::unique_ptr<Image<RgbPixel>> c_brig = graybrig->Clone();
  std::unique_ptr<Image<RgbPixel>> c_dark = graydark->Clone();
  float lap_kernel[3] = {1.f, -2.f, 1.f};
  convolve(c_dark, width, height, 3, lap_kernel, true);
  convolve(c_brig, width, height, 3, lap_kernel, true);

  // Saturation - standard deviation
  //
  std::unique_ptr<Image<RgbPixel>> s_brig = rgbbrig_luma->Clone();
  std::unique_ptr<Image<RgbPixel>> s_dark = rgbdark_luma->Clone();
  for (size_t row = 0; row < height; ++row) {
    for (size_t col = 0; col < width; ++col) {
      auto& pixelbrig = (*rgbbrig_luma)(row, col);
      auto& pixeldark = (*rgbdark_luma)(row, col);

      (*s_brig)(row, col).r = stdev3(pixelbrig.r, pixelbrig.g, pixelbrig.b);
      (*s_brig)(row, col).g = (*s_brig)(row, col).b = (*s_brig)(row, col).r;
      (*s_dark)(row, col).r = stdev3(pixeldark.r, pixeldark.g, pixeldark.b);
      (*s_dark)(row, col).g = (*s_dark)(row, col).b = (*s_dark)(row, col).r;
    }
  }

  // Well-exposedness
  //
  std::unique_ptr<Image<RgbPixel>> e_brig = rgbbrig->Clone();
  std::unique_ptr<Image<RgbPixel>> e_dark = rgbdark->Clone();
  for (size_t row = 0; row < height; ++row) {
    for (size_t col = 0; col < width; ++col) {
      auto& pixelbrig = (*rgbbrig)(row, col);
      auto& pixeldark = (*rgbdark)(row, col);

      (*e_brig)(row,col).r = (*e_brig)(row,col).g = (*e_brig)(row,col).b =
        gausscurve(pixelbrig.r) * gausscurve(pixelbrig.g) *
        gausscurve(pixelbrig.b);
      (*e_dark)(row,col).r = (*e_dark)(row,col).g = (*e_dark)(row,col).b =
        gausscurve(pixeldark.r) * gausscurve(pixeldark.g) *
        gausscurve(pixeldark.b);
    }
  }

  // Mix the 3 weight
  std::unique_ptr<Image<RgbPixel>> w_brig = rgbbrig->Clone();
  std::unique_ptr<Image<RgbPixel>> w_dark = rgbdark->Clone();
  for (size_t row = 0; row < height; ++row) {
    for (size_t col = 0; col < width; ++col) {
      auto& pixelbrig = (*w_brig)(row, col);
      auto& pixeldark = (*w_dark)(row, col);
      pixelbrig.r = (*c_brig)(row,col).r * (*s_brig)(row,col).r * (*e_brig)(row,col).r;
      pixeldark.r = (*c_dark)(row,col).r * (*s_dark)(row,col).r * (*e_dark)(row,col).r;

      pixelbrig.r = pixelbrig.g = pixelbrig.b = pixelbrig.r / (pixelbrig.r +
                                                               pixeldark.r + 1.0e-10f);
      pixeldark.r = pixeldark.g = pixeldark.b = 1.f - pixelbrig.r;

    }
  }

  auto t8 = chrono::steady_clock::now();
  cout << " Runtime: " 
       << chrono::duration_cast<chrono::milliseconds>(t8 - t7).count()
       << " ms" << endl;
  // Calculating the pyramid
  cout << "Calculating Gaussian and Laplacian pyramid...";
  int depth = 5;
  std::vector<std::unique_ptr<Image<RgbPixel>>> LP_brig;
  std::vector<std::unique_ptr<Image<RgbPixel>>> LP_dark;
  lp_pyramid(rgbbrig, LP_brig, width, height, depth);
  lp_pyramid(rgbdark, LP_dark, width, height, depth);

  std::vector<std::unique_ptr<Image<RgbPixel>>> GP_w_brig;
  std::vector<std::unique_ptr<Image<RgbPixel>>> GP_w_dark;
  // Just downsample, don't gauss
  gp_pyramid(w_brig, GP_w_brig, width, height, depth, !false);
  gp_pyramid(w_dark, GP_w_dark, width, height, depth, !false);

  // Try to fuse the pyramid
  std::vector<std::unique_ptr<Image<RgbPixel>>> LP_fuse;
  lp_fuse(LP_fuse, LP_brig, GP_w_brig, LP_dark, GP_w_dark, width, height);
  lp_reconstruct(LP_fuse, width, height);

  auto t9 = chrono::steady_clock::now();
  cout << " Runtime: " 
       << chrono::duration_cast<chrono::milliseconds>(t9 - t8).count()
       << " ms" << endl;
  // For visualization of the grayscale.
  //
  cout << "visualization in progress...";
  for (size_t row = 0; row < height; ++row) {
    for (size_t col = 0; col < width; ++col) {
      auto& pixel = (*LP_fuse[0])(row, col);
      // auto& pixel = (*w_dark)(row, col);
      pixel = pixel.pow(pixel,0.44f) * 255.f;

    }
  }

  auto t10 = chrono::steady_clock::now();
  cout << "Total Runtime: " 
       << chrono::duration_cast<chrono::seconds>(t10 - t1).count()
       << " seconds" << endl;

  // return processed image output
  // return std::move(w_dark);
  return std::move(LP_fuse[0]);

  // END: CS348K STUDENTS MODIFY THIS CODE  
}


// Helper function
//

// Convolve the image with a kernel
//
void CameraPipeline::convolve(std::unique_ptr<Image<RgbPixel>>& image, 
                              int width, int height, int kernel_length,
                              float* kernel, bool is_abs) const
{
  std::unique_ptr<Image<RgbPixel>> tmpimg(new Image<RgbPixel>(width, height));
  int shift = kernel_length / 2;
  float largest = -1000.f;
  // Perform row convolution
  for (size_t row = 0; row < height; ++row) {
    for (size_t col = 0; col < width; ++col) {
      for (size_t k_idx = 0; k_idx < kernel_length; ++k_idx) {
        (*tmpimg)(row, col) += (*image)(row, miridx(col + k_idx - shift, width)) *
                                 kernel[k_idx];
      }
    }
  }

  // Perform column convolution
  for (size_t row = 0; row < height; ++row) {
    for (size_t col = 0; col < width; ++col) {
      (*image)(row, col).r = 0.f;
      (*image)(row, col).g = 0.f;
      (*image)(row, col).b = 0.f;
      for (size_t k_idx = 0; k_idx < kernel_length; ++k_idx) {
        (*image)(row, col) += (*tmpimg)(miridx(row + k_idx - shift, height), col) *
                                kernel[k_idx];
      }
    }
  }

  if (is_abs) {
    for (size_t row = 0; row < height; ++row) {
      for (size_t col = 0; col < width; ++col) {
        auto& pixel = (*image)(row, col);
        pixel.r = abs(pixel.r);
        pixel.g = abs(pixel.g);
        pixel.b = abs(pixel.b);
      }
    }
  }
}

// Compute the Gaussian pyramid
// We perform gaussian blur and downsample (by 2) for [depth] number of times.
//
void CameraPipeline::gp_pyramid(std::unique_ptr<Image<RgbPixel>>& image,
                                std::vector<std::unique_ptr<Image<RgbPixel>>>& out,
                                int width, int height, int depth, bool is_gauss) const
{
  std::unique_ptr<Image<RgbPixel>> curimg = image->Clone();
  std::unique_ptr<Image<RgbPixel>> nextimg;
  float gauss_kernel[5] = {0.06136, 0.24477, 0.38774, 0.24477, 0.06136};
  int cur_width = width;
  int cur_height = height;

  for (size_t num = 0; num < depth; ++num) {
    // Perform the blur to the current image
    std::unique_ptr<Image<RgbPixel>> tmpimg = curimg->Clone();
    if (is_gauss)
      convolve(tmpimg, cur_width, cur_height, 5, gauss_kernel);

    // Prepare the next image, halve the width and height
    cur_width /= 2;
    cur_height /= 2;
    nextimg = std::unique_ptr<Image<RgbPixel>>(new Image<RgbPixel>(cur_width, cur_height));

    // Perform the downsampling
    for (size_t row = 0; row < cur_height; ++row) {
      for (size_t col = 0; col < cur_width; ++col) {
        (*nextimg)(row,col) = (*tmpimg)(row*2,col*2);
      }
    }

    out.push_back(std::move(curimg));
    curimg = std::move(nextimg);
  }
}

// Compute the Laplacian pyramid
// We calculate the gaussian pyramid, and perform upsampling to calculate the
// Laplacian pyramid.
//
void CameraPipeline::lp_pyramid(std::unique_ptr<Image<RgbPixel>>& image,
                                std::vector<std::unique_ptr<Image<RgbPixel>>>& out,
                                int width, int height, int depth) const
{
  // We first calculate the gaussian pyramid
  //
  gp_pyramid(image, out, width, height, depth);

  // Upsampling 2x2 kernel. Multiply by 2, so the output is multiplied by 4.
  //
  float gauss_kernel[5] = {2*0.06136, 2*0.24477, 2*0.38774, 2*0.24477, 2*0.06136};

  int cur_width = width;
  int cur_height = height;

  // We upsampling the image, and calculate the difference.
  //
  for (int num = 0; num < depth - 1; ++num) {
    // Upsampling the image.
    //
    std::unique_ptr<Image<RgbPixel>> tmpimg(new Image<RgbPixel>(cur_width, cur_height));
    for (size_t row = 0; row < cur_height/2; ++row) {
      for (size_t col = 0; col < cur_width/2; ++col) {
        (*tmpimg)(row*2,col*2) = (*out[num+1])(row,col);
      }
    }
    convolve(tmpimg, cur_width, cur_height, 5, gauss_kernel);

    // Subtract and calculate the Laplacian pyramid
    //
    for (size_t row = 0; row < cur_height; ++row) {
      for (size_t col = 0; col < cur_width; ++col) {
        (*out[num])(row,col) = (*out[num])(row,col) - (*tmpimg)(row,col);
      }
    }

    cur_height /= 2;
    cur_width /= 2;
  }
}

void CameraPipeline::lp_reconstruct(std::vector<std::unique_ptr<Image<RgbPixel>>>& out,
                                    int width, int height) const
{
  int depth = out.size();
  float gauss_kernel[5] = {2*0.06136, 2*0.24477, 2*0.38774, 2*0.24477, 2*0.06136};

  int cur_width = width / pow(2, depth-2);
  int cur_height = height / pow(2, depth-2);

  for (int num = depth - 2; num >=0; --num) {
    std::unique_ptr<Image<RgbPixel>> tmpimg(new Image<RgbPixel>(cur_width, cur_height));
    for (size_t row = 0; row < cur_height/2; ++row) {
      for (size_t col = 0; col < cur_width/2; ++col) {
        (*tmpimg)(row*2,col*2) = (*out[num+1])(row,col);
      }
    }
    convolve(tmpimg, cur_width, cur_height, 5, gauss_kernel);

    for (size_t row = 0; row < cur_height; ++row) {
      for (size_t col = 0; col < cur_width; ++col) {
        (*out[num])(row,col) = (*out[num])(row,col) + (*tmpimg)(row, col);
      }
    }

    cur_width *= 2;
    cur_height *= 2;
  }
}

void CameraPipeline::lp_fuse(std::vector<std::unique_ptr<Image<RgbPixel>>>& out,
                             std::vector<std::unique_ptr<Image<RgbPixel>>>& lp1,
                             std::vector<std::unique_ptr<Image<RgbPixel>>>& gp1,
                             std::vector<std::unique_ptr<Image<RgbPixel>>>& lp2,
                             std::vector<std::unique_ptr<Image<RgbPixel>>>& gp2,
                             int width, int height) const
{
  int depth = lp1.size();
  int cur_width = width;
  int cur_height = height;

  for (int num = 0; num < depth; ++num) {
    std::unique_ptr<Image<RgbPixel>> tmpimg(new Image<RgbPixel>(cur_width, cur_height));
    for (size_t row = 0; row < cur_height; ++row) {
      for (size_t col = 0; col < cur_width; ++col) {
        (*tmpimg)(row,col) = (*lp1[num])(row,col) * (*gp1[num])(row,col).r +
                             (*lp2[num])(row,col) * (*gp2[num])(row,col).r;
      }
    }
    out.push_back(std::move(tmpimg));
    cur_width /= 2;
    cur_height /= 2;
  }
}

void CameraPipeline::gp_align(std::vector<std::unique_ptr<Image<RgbPixel>>>& gp1,
                              std::vector<std::unique_ptr<Image<RgbPixel>>>& gp2,
                              std::unique_ptr<Image<RgbPixel>>& out,
                              int width, int height, int depth,
                              int tile, int stride, int radi) const
{ 
  std::unique_ptr<Image<RgbPixel>> last_offset;
  for (int num = depth - 1; num >= 0; --num) {
    float average_diff = 0.f;
    int cur_width = width / pow(2, num);
    int cur_height = height / pow(2, num);
    int offset_width = (cur_width - tile) / stride + 1;
    int offset_height = (cur_height - tile) / stride + 1;
    float up_kernel[5] = {2*0.06136, 2*0.24477, 2*0.38774, 2*0.24477, 2*0.06136};
    std::unique_ptr<Image<RgbPixel>> offset(new Image<RgbPixel>(offset_width,
                                                                offset_height));
    // Upsample the offset from the last aligning results.
    if (num < depth - 1) {
      int last_offset_width = (cur_width / 2 - tile) / stride + 1;
      int last_offset_height = (cur_height / 2 - tile) / stride + 1;
      for (int row = 0; row < last_offset_height; ++row) {
        for (int col = 0; col < last_offset_width; ++col) {
          (*offset)(row*2,col*2) = (*last_offset)(row,col);
        }
      }
      convolve(offset, offset_width, offset_height, 5, up_kernel);
      // Round the offset
      for (int row = 0; row < offset_height; ++row) {
        for (int col = 0; col < offset_width; ++col) {
          (*offset)(row,col).r = round((*offset)(row,col).r);
          (*offset)(row,col).g = round((*offset)(row,col).g);
        }
      }
    }
    // For each tile in the image.
    for (int row = 0; row < cur_height - tile; row += stride) {
      for (int col = 0; col < cur_width - tile; col += stride) {
        float min_dist = 1.0e10;
        int row_offset = 0;
        int col_offset = 0;

        // The offset from the last aligning results.
        //
        int last_offset_row = 0;
        int last_offset_col = 0;
        if (num != depth - 1) {
          last_offset_row = (int)(*offset)(row/stride, col/stride).r;
          last_offset_col = (int)(*offset)(row/stride, col/stride).g;
        }

        // For each neighbouring tile.
        for (int tilerow = -radi; tilerow <= radi; ++tilerow) {
          for (int tilecol = -radi; tilecol <= radi; ++tilecol) {
            // check if the neighbour is valid
            if ((row + tilerow + last_offset_row < 0) || 
                (row + tilerow + last_offset_row + tile >= cur_height) ||
                (col + tilecol + last_offset_col < 0) || 
                (col + tilecol + last_offset_col + tile >= cur_width))
              continue;

            float total_dist = 0.f;
            // For each pixel in the tile
            for (int tilex = 0; tilex < tile; ++tilex) {
              for (int tiley = 0; tiley < tile; ++tiley) {
                int row_num = row + last_offset_row + tilerow + tilex;
                int col_num = col + last_offset_col + tilecol + tiley;
                total_dist += abs((*gp1[num])(row+tilex, col+tiley).r -
                              (*gp2[num])(row_num, col_num).r);
              }
            }
            // End of accumulation
            // Update the minimum distance
            //

            if (total_dist < min_dist) {
              min_dist = total_dist;
              row_offset = tilerow + last_offset_row;
              col_offset = tilecol + last_offset_col;
            }
          }
        }
        // End of finding minimum
        (*offset)(row/stride, col/stride).r = (float)row_offset;
        (*offset)(row/stride, col/stride).g = (float)col_offset;
        (*offset)(row/stride, col/stride).b = (float)min_dist;
        average_diff += min_dist;
      }
    }
    // End of a image
    last_offset = std::move(offset);
    average_diff /= (offset_width * offset_height);
    // cout << "average_diff for depth " << num << " is: " << average_diff << endl;
  }
  out = std::move(last_offset);
}

