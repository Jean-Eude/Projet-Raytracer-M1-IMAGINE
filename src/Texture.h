#ifndef TEXTURE_H
#define TEXTURE_H

#include "imageLoader.h"
#include "Vec3.h"
#include <cmath>
#include <string>
#include <random>

float whitenoise() {
    static std::mt19937 generator(std::random_device{}());
    static std::uniform_real_distribution<float> distribution(0.0, 1.0);
    return distribution(generator);
}

class Texture {
  public:
    float u, v;
    Vec3 color;

    Texture(float u, float v) : u(u), v(v) {}

    void setConstantColor(Vec3 color) {
        this->color = color;
    }

    void setCheckeredTexture(int nbRepetitions) {
        int x = floor(u * nbRepetitions);
        int y = floor(v * nbRepetitions);

        if ((x + y) % 2 == 0) {
            this->color = Vec3(1.0f, 1.0f, 1.0f); 
        } else {
            this->color = Vec3(0.0f, 0.0f, 0.0f);
        }
    } 

    void setWhiteNoiseTexture() {
        this->color = Vec3(whitenoise(), whitenoise(), whitenoise());
    } 

    void setTextureFromFilePPM(const std::string& filename) {
        ppmLoader::ImageRGB img; 
        ppmLoader::load_ppm(img, filename);

        int i = floor(u*img.w);
        int j = floor((1-v)*img.h); // Inversion des coordonnées pour avoir la texture à l'endroit

        if (i < 0) i = 0;
        if (j < 0) j = 0;

        if (i > img.w - 1) i = img.w - 1;
        if (j > img.h - 1) j = img.h - 1;

        int indice = (i+img.w*j);
        std::cout << indice << std::endl;
        ppmLoader::RGB a = img.data[indice];
        this->color = Vec3((float)a.r/255.0, (float)a.g/255.0, (float)a.b/255.0);
    }
};


#endif