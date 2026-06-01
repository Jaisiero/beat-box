#include "image.hpp"
#include <iostream>

#define STB_IMAGE_IMPLEMENTATION
#define STBI_FAILURE_USERMSG
#include <stb_image.h>

BB_NAMESPACE_BEGIN

BBImage::BBImage(std::string filename, const char* path)
{
    if (load(std::string(path) + "/" + filename))
        return;

    static bool warned = false;
    if (!warned) {
        std::cerr << "WARNING: Could not load image files (e.g. STBN). Using dummy zero-initialized textures instead.\n";
        warned = true;
    }

    image_width = 128;
    image_height = 128;
    bytes_per_scanline = 128 * bytes_per_pixel;
    data = (unsigned char*)std::malloc(image_width * image_height * bytes_per_pixel);
    if (data) {
        std::memset(data, 0, image_width * image_height * bytes_per_pixel);
    }
}

BBImage::~BBImage()
{
    stbi_image_free(data);
}
bool BBImage::load(const std::string filename) {
  // Loads image data from the given file name. Returns true if the load succeeded.
  auto n = bytes_per_pixel; // Dummy out parameter: original components per pixel
  data = stbi_load(filename.c_str(), &image_width, &image_height, &n, bytes_per_pixel);
  bytes_per_scanline = image_width * bytes_per_pixel;
  return data != nullptr;
}


BB_NAMESPACE_END