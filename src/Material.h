#ifndef MATERIAL_H
#define MATERIAL_H

#include "imageLoader.h"
#include "Vec3.h"
#include <cmath>

#include <GL/glut.h>

enum MaterialType {
    Material_Texture,
    Material_PBR,
    Material_FresnelRim,
    Material_Toon,
    Material_Blinn_Phong,
    Material_Phong,
    Material_Glass,
    Material_Mirror,
    Material_Basic
};

enum TextureType {
    Texture_Color,
    Texture_Checkered,
    Texture_WhiteNoise,
    Texture_File
};


struct Material {
    Vec3 ambient_material;
    Vec3 diffuse_material;
    Vec3 specular_material;
    double shininess;

    float index_medium;
    float transparency;

    MaterialType type;
    TextureType textype;
    int nbRepetitions;

    Material() {
        type = Material_Phong;
        transparency = 0.0;
        index_medium = 1.0;
        ambient_material = Vec3(0., 0., 0.);
    }
};



#endif // MATERIAL_H
