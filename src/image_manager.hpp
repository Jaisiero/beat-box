#pragma once

#include "defines.hpp"
#include "math.hpp"

#define ASSETS_PATH "assets/"
#define TEXTURES_PATH ASSETS_PATH "textures/"
#define SPATIOTEMPORAL_BLUE_NOISE_PATH TEXTURES_PATH "STBN/"

BB_NAMESPACE_BEGIN

struct BBImage
{
public:

    BBImage() : data(nullptr) {};

    explicit BBImage(const char *image_filename, const char* path);
    
    ~BBImage();

    int width()  const { return (data == nullptr) ? 0 : image_width; }
    int height() const { return (data == nullptr) ? 0 : image_height; }
    unsigned int size() const { return image_width * image_height * bytes_per_pixel; }

    const unsigned char* pixel_data(int x, int y) const {
        // Return the address of the three bytes of the pixel at x,y (or magenta if no data).
        static unsigned char magenta[] = { 255, 0, 255 };
        if (data == nullptr) return magenta;

        x = clamp(x, 0, image_width);
        y = clamp(y, 0, image_height);

        return data + y*bytes_per_scanline + x*bytes_per_pixel;
    }

    const unsigned char* data_ptr() const { return data; }

  private:

    bool load(const std::string filename);

    const int bytes_per_pixel = 4; // RGBA format cause the fact that many GPUs doesn't support RGB
    unsigned char *data;
    int image_width, image_height;
    int bytes_per_scanline;

    static int clamp(int x, int low, int high) {
        // Return the value clamped to the range [low, high).
        if (x < low) return low;
        if (x < high) return x;
        return high - 1;
    }
};

class BBAbstractTexture {
public:
    virtual ~BBAbstractTexture () = default;
    virtual std::unique_ptr<BBAbstractTexture> clone() const = 0;
};

template <typename Derived>
class BBTexture : public BBAbstractTexture {
public:
    std::unique_ptr<BBAbstractTexture> clone() const override {
        return std::make_unique<Derived>(static_cast<Derived const&>(*this));
    }

    virtual daxa_f32vec3 value(float u, float v, const daxa_f32vec3& p) const = 0;

// protected:
   // We make clear BBTexture class needs to be inherited
   BBTexture() = default;
   BBTexture(const BBTexture&) = default;
   BBTexture(BBTexture&&) = default;
};

class BBImageTexture : public BBTexture<BBImageTexture> {
  public:

    BBImageTexture() = default;

    BBImageTexture(const char* filename, const char* path = TEXTURES_PATH) : image(filename, path) {}

    daxa_f32vec3 value(float u, float v, const daxa_f32vec3& p) const override {
        // If we have no texture data, then return solid cyan as a debugging aid.
        if (image.height() <= 0) return daxa_f32vec3(0,1,1);

        // Clamp input texture coordinates to [0,1] x [1,0]
        u = interval(0,1).clamp(u);
        v = 1.0 - interval(0,1).clamp(v);  // Flip V to image coordinates

        auto i = static_cast<int>(u * image.width());
        auto j = static_cast<int>(v * image.height());
        auto pixel = image.pixel_data(i,j);

        auto color_scale = 1.0 / 255.0;
        return daxa_f32vec3(color_scale*pixel[0], color_scale*pixel[1], color_scale*pixel[2]);
    }

    unsigned int get_size() const {
        return image.size();
    }

    unsigned int get_width() const {
        return image.width();
    }

    unsigned int get_height() const {
        return image.height();
    }

    const unsigned char* get_data() const {
        return image.data_ptr();
    }

    ~BBImageTexture() = default;

  private:
    BBImage image;
};

BB_NAMESPACE_END