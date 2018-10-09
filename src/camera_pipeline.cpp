#include "camera_pipeline.hpp"


std::unique_ptr<Image<RgbPixel>> CameraPipeline::ProcessShot() const {
    
  // BEGIN: CS348K STUDENTS MODIFY THIS CODE

  // put the lens cap on if you'd like to measure a "dark frame"
  sensor_->SetLensCap(false);
    
  // grab RAW pixel data from sensor
  const int width = sensor_->GetSensorWidth();
  const int height = sensor_->GetSensorHeight();
  auto raw_data = sensor_->GetSensorData(0, 0, width, height);
    
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

  // Calculate the bright difference on the 3x3 tile.
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
  // https://www.ingentaconnect.com/contentone/ist/ei/2017/00002017/00000015/art00008?crawler=true&mimetype=application/pdf
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

  cout << "hot: " << cnt1 << " cold: " << cnt2 << endl;

  // We first perform demosaicing by 2x2 tile bilinear
  //
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

  // Perform a Gaussian blur to the image
  // by defining the Gaussian kernel.
  float kernel[5] = {0.06136, 0.24477, 0.38774, 0.24477, 0.06136};
  std::unique_ptr<Image<RgbPixel>> tmpimg(new Image<RgbPixel>(width, height));

  // Perform row convolution
  for (size_t row = 0; row < height; ++row) {
    for (size_t col = 0; col < width; ++col) {
      for (size_t k_idx = 0; k_idx < 5; ++k_idx) {
        (*tmpimg)(row, col) += (*image)(row, miridx(col + k_idx - 2, width)) *
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
      for (size_t k_idx = 0; k_idx < 5; ++k_idx) {
        (*image)(row, col) += (*tmpimg)(miridx(row + k_idx - 2, height), col) *
                                kernel[k_idx];
      }
    }
  }

  // Rescale the [0~1] floating number to [0~255]
  // Use a tangent mapping derived by myself.
  for (size_t row = 0; row < height; ++row) {
    for (size_t col = 0; col < width; ++col) {
      auto& pixel = (*image)(row, col);
      // pixel.r = tan_map(pixel.r);
      // pixel.g = tan_map(pixel.g);
      // pixel.b = tan_map(pixel.b);
      // pixel.r *= 255.f;
      // pixel.g *= 255.f;
      // pixel.b *= 255.f;
      pixel.r = pow(pixel.r, 0.44) * 255.f;
      pixel.g = pow(pixel.g, 0.44) * 255.f;
      pixel.b = pow(pixel.b, 0.44) * 255.f;
    }
  }

  /*
   This is the original code

  // allocate 3-channel RGB output buffer to hold the results after processing 
  std::unique_ptr<Image<RgbPixel>> image(new Image<RgbPixel>(width, height));
  
  // The starter code copies the raw data from the sensor to all rgb
  // channels. This results in a gray image that is just a
  // visualization of the sensor's contents.
  
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      const auto val = raw_data->data(row, col);
      auto& pixel = (*image)(row, col);

      // pixel data from the sensor is normalized to the 0-1 range, so
      // scale by 255 for the final image output.  Output image pixels
      // should be in the 0-255 range.
      pixel.r = val * 255.f;
      pixel.g = val * 255.f;
      pixel.b = val * 255.f;
    }
  }
  */
  
  // return processed image output
  return image;

  // END: CS348K STUDENTS MODIFY THIS CODE  
}
