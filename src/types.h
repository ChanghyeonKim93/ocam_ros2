#ifndef TYPES_H_
#define TYPES_H_

#include <cstdint>
#include <vector>

namespace ocam {

struct ImageSize {
  int width{0};
  int height{0};
};

struct Image {
  enum class ImageType { GRAY8, RGB8, BGR8, RGBA8, BGRA8 };
  ImageSize image_size;
  int step{0};
  std::vector<uint8_t> data;
};

}  // namespace ocam

#endif  // TYPES_H_
