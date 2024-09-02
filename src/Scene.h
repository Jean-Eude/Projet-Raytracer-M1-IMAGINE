#ifndef SCENE_H
#define SCENE_H

#include <vector>
#include <string>
#include "Mesh.h"
#include "Sphere.h"
#include "Square.h"
#include "Triangle.h"
#include <cmath>
#include "Texture.h"
#include "Maths.h"
#include "PBR.h"
#include "Camera.h"
#include "AABB.h"

#include <GL/glut.h>


enum LightType {
    LightType_Spherical,
    LightType_Quad
};


struct Light {
    Vec3 material;
    bool isInCamSpace;
    LightType type;

    Vec3 pos;
    float radius;

    Mesh quad;

    float powerCorrection;

    Light() : powerCorrection(1.0) {}
};

struct RaySceneIntersection{
    bool intersectionExists;
    unsigned int typeOfIntersectedObject;
    unsigned int objectIndex;
    float t;
    RayTriangleIntersection rayMeshIntersection;
    RaySphereIntersection raySphereIntersection;
    RaySquareIntersection raySquareIntersection;
    RaySceneIntersection() : intersectionExists(false) , t(FLT_MAX) {}
};


class Scene {
    std::vector< Mesh > meshes;
    std::vector< Sphere > spheres;
    std::vector< Square > squares;
    std::vector< Light > lights;

    std::vector< AABB > bbox;

    Camera camera;


public:


    Scene() {
    }

    void draw() {
        // iterer sur l'ensemble des objets, et faire leur rendu :
        for( unsigned int It = 0 ; It < spheres.size() ; ++It ) {
            Sphere const & sphere = spheres[It];
            sphere.draw();
        }
        for( unsigned int It = 0 ; It < squares.size() ; ++It ) {
            Square const & square = squares[It];
            square.draw();
        }
        for( unsigned int It = 0 ; It < meshes.size() ; ++It ) {
            Mesh const & mesh = meshes[It];
            mesh.draw();
        }
    }


    RaySceneIntersection computeIntersection(Ray const & ray) {
        RaySceneIntersection result;
        //TODO calculer les intersections avec les objets de la scene et garder la plus proche

        for(int It = 0 ; It < spheres.size() ; It++) {
            Sphere const & sphere = spheres[It];
            RaySphereIntersection raySphereIntersection = sphere.intersect(ray);
            if(raySphereIntersection.intersectionExists && raySphereIntersection.t < result.t) {    // On garde l'intersection la plus proche et on vérifie que l'intersection existe
                result.typeOfIntersectedObject = 0;     // 0 pour une sphère
                result.objectIndex = It;
                result.t = raySphereIntersection.t;
                result.intersectionExists = raySphereIntersection.intersectionExists;
                result.raySphereIntersection = raySphereIntersection;
            }
        }

        for(int It = 0 ; It < squares.size() ; It++) {
            Square const & square = squares[It];
            RaySquareIntersection raySquareIntersection = square.intersect(ray);
            if(raySquareIntersection.intersectionExists && raySquareIntersection.t < result.t) {      // On garde l'intersection la plus proche et on vérifie que l'intersection existe
                result.typeOfIntersectedObject = 1;     // 1 pour un carré
                result.objectIndex = It;
                result.t = raySquareIntersection.t;
                result.intersectionExists = raySquareIntersection.intersectionExists;
                result.raySquareIntersection = raySquareIntersection;
            }
        }

        // Avec KD-Tree

        for(int bboxIndex = 0; bboxIndex < bbox.size(); bboxIndex++)
        {
            if(bbox[bboxIndex].intersect(ray))
            {
                for (int It = 0; It < meshes.size(); It++)
                {
                    RayTriangleIntersection mesh = meshes[It].intersect(ray);
                    if(mesh.intersectionExists)
                    {
                        if(mesh.t < result.t && mesh.t < camera.getFarPlane() && mesh.t > camera.getNearPlane())
                        {
                            result.typeOfIntersectedObject = 2;     // 2 pour un mesh
                            result.objectIndex = It;
                            result.t = mesh.t;
                            result.intersectionExists = mesh.intersectionExists;
                            result.rayMeshIntersection = mesh;   
                        }
                    }
                }
            }
        }
        

        // Sans KD-Tree
        /*
        for(int It = 0 ; It < meshes.size() ; It++) {
            Mesh const & mesh = meshes[It];
            RayTriangleIntersection rayTriangleIntersection = mesh.intersect(ray);
            if(rayTriangleIntersection.intersectionExists && rayTriangleIntersection.t < result.t) {      // On garde l'intersection la plus proche et on vérifie que l'intersection existe
                result.typeOfIntersectedObject = 2;     // 2 pour un mesh
                result.objectIndex = It;
                result.t = rayTriangleIntersection.t;
                result.intersectionExists = rayTriangleIntersection.intersectionExists;
                result.rayMeshIntersection = rayTriangleIntersection;
            }
        }
        */
        return result;
    }


    void ComputeShadowsHardAndSoft_Sphere(std::vector<Light> &lights, int &i, RaySceneIntersection &raySceneIntersection, bool &estDansOmbre, Vec3 &L, float &marge, float &shadow, int &nbRayonsOmbres, float &tailleCarre, Vec3 &intensiteDS) {
        // Ombres dures            
        if(lights[i].type == LightType_Spherical) {
            Ray shadowRay(raySceneIntersection.raySphereIntersection.intersection + marge * L, L);
            RaySceneIntersection shadowIntersection = computeIntersection(shadowRay);
            if(shadowIntersection.intersectionExists == true) {
                if(shadowIntersection.t < L.length()) {
                    estDansOmbre = true;
                } else {
                    estDansOmbre = false;                               
                }
            }                       
        }

        // Ombres douces 
        else if(lights[i].type == LightType_Quad) {
            float counter = 0;
            shadow = 0.0f;

            for(float i = 0; i < nbRayonsOmbres; i++) {
                Vec3 Y = Vec3(0.0f,1.0f,0.0f);
                Vec3 X = Vec3(1.0f,0.0f,0.0f);

                // Echantillonnage du carré
                X *= (tailleCarre / 2.);
                Y *= (tailleCarre / 2.);
                float u = (float)rand() / (float)(RAND_MAX) - 0.5f;  // Meilleur échantillonnage
                float v = (float)rand() / (float)(RAND_MAX) - 0.5f;  // Meilleur échantillonnage
                Vec3 PositionPoint = lights[i].pos + (u * X) + (v * Y);
                PositionPoint.normalize();

                Vec3 LightL = PositionPoint - raySceneIntersection.raySphereIntersection.intersection;
                LightL.normalize();
                Ray shadowRay = Ray(raySceneIntersection.raySphereIntersection.intersection + marge * LightL, LightL);
                RaySceneIntersection shadowIntersection = computeIntersection(shadowRay);

                if(shadowIntersection.t < LightL.length()) {
                    counter++;
                    estDansOmbre = true;
                } else {
                    estDansOmbre = false;                               
                }
            }

            shadow = 1.0f - (float)counter/(float)nbRayonsOmbres;
            intensiteDS *= shadow;
        }
    }


    void ComputeShadowsHardAndSoft_Square(std::vector<Light> &lights, int &i, RaySceneIntersection &raySceneIntersection, bool &estDansOmbre, Vec3 &L, float &marge, float &shadow, int &nbRayonsOmbres, float &tailleCarre, Vec3 &intensiteDS) {
        // Ombres dures            
        if(lights[i].type == LightType_Spherical) {
            Ray shadowRay(raySceneIntersection.raySquareIntersection.intersection + marge * L, L);
            RaySceneIntersection shadowIntersection = computeIntersection(shadowRay);
            if(shadowIntersection.intersectionExists == true) {
                if(shadowIntersection.t < L.length()) {
                    estDansOmbre = true;
                } else {
                    estDansOmbre = false;                               
                }
            }
        } 

        // Ombres douces            
        else if(lights[i].type == LightType_Quad) {
            float counter = 0;
            shadow = 0.0f;

            for(float i = 0; i < nbRayonsOmbres; i++) {
                Vec3 Y = Vec3(0.0f,1.0f,0.0f);
                Vec3 X = Vec3(1.0f,0.0f,0.0f);

                // Echantillonnage du carré
                X *= (tailleCarre / 2.);
                Y *= (tailleCarre / 2.);

                float u = (float)rand() / (float)(RAND_MAX) - 0.5f;  // Meilleur échantillonnage
                float v = (float)rand() / (float)(RAND_MAX) - 0.5f;  // Meilleur échantillonnage
                Vec3 PositionPoint = lights[i].pos + (u * X) + (v * Y);
                PositionPoint.normalize();

                Vec3 LightL = PositionPoint - raySceneIntersection.raySquareIntersection.intersection;
                LightL.normalize();
                Ray shadowRay = Ray(raySceneIntersection.raySquareIntersection.intersection + marge * LightL, LightL);
                RaySceneIntersection shadowIntersection = computeIntersection(shadowRay);

                if(shadowIntersection.t < LightL.length()) {
                    counter++;
                    estDansOmbre = true;
                } else {
                    estDansOmbre = false;                               
                }
            }

            shadow = 1.0f - (float)counter/(float)nbRayonsOmbres;
            intensiteDS *= shadow;
        }
    }

    void ComputeShadowsHardAndSoft_Mesh(std::vector<Light> &lights, int &i, RaySceneIntersection &raySceneIntersection, bool &estDansOmbre, Vec3 &L, float &marge, float &shadow, int &nbRayonsOmbres, float &tailleCarre, Vec3 &intensiteDS) {
        // Ombres dures            
        if(lights[i].type == LightType_Spherical) {
            Ray shadowRay(raySceneIntersection.rayMeshIntersection.intersection + marge * L, L);
            RaySceneIntersection shadowIntersection = computeIntersection(shadowRay);
            if(shadowIntersection.intersectionExists == true) {
                if(shadowIntersection.t < L.length()) {
                    estDansOmbre = true;
                } else {
                    estDansOmbre = false;                               
                }
            }
        }
        // Ombres douces             
        else if(lights[i].type == LightType_Quad) {
            float counter = 0;
            shadow = 0.0f;

            for(float i = 0; i < nbRayonsOmbres; i++) {
                Vec3 Y = Vec3(0.0f,1.0f,0.0f);
                Vec3 X = Vec3(1.0f,0.0f,0.0f);

                // Echantillonnage du carré
                X *= (tailleCarre / 2.);
                Y *= (tailleCarre / 2.);
                float u = (float)rand() / (float)(RAND_MAX) - 0.5f;  // Meilleur échantillonnage
                float v = (float)rand() / (float)(RAND_MAX) - 0.5f;  // Meilleur échantillonnage
                Vec3 PositionPoint = lights[i].pos + (u * X) + (v * Y);
                PositionPoint.normalize();

                Vec3 LightL = PositionPoint - raySceneIntersection.rayMeshIntersection.intersection;
                LightL.normalize();
                Ray shadowRay = Ray(raySceneIntersection.rayMeshIntersection.intersection + marge * LightL, LightL);
                RaySceneIntersection shadowIntersection = computeIntersection(shadowRay);

                if(shadowIntersection.t < L.length()) {
                    counter++;
                    estDansOmbre = true;
                } else {
                    estDansOmbre = false;                               
                }
            }

            shadow = 1.0f - (float)counter/(float)nbRayonsOmbres;
            intensiteDS *= shadow;
        }
    }


    // Basic Material
    void ComputeBasicMaterial_Sphere(Vec3 &color, Sphere const & sphere) {
        color = sphere.material.diffuse_material;
    }

    void ComputeBasicMaterial_Square(Vec3 &color, Square const & square) {
        color = square.material.diffuse_material;
    }

    void ComputeBasicMaterial_Mesh(Vec3 &color, Mesh const & mesh) {
        color = mesh.material.diffuse_material;
    }


    // Phong Material
    void ComputePhongMaterial_Sphere(Vec3 &color, Sphere &sphere,RaySceneIntersection &raySceneIntersection, Ray &ray, Vec3 &ambiant, float intensiteAmbiant, Vec3 &diffuse, float intensiteDiffuse, Vec3 &speculaire, float intensiteSpeculaire, Vec3 &intensiteDS, Vec3 &L, Vec3 &R, Vec3 &V,  bool &estDansOmbre, float &shadow, int &nbRayonsOmbres, float &tailleCarre, float marge) {
        Vec3 Normal = raySceneIntersection.raySphereIntersection.normal;
        Normal.normalize();

        // Réfléxion ambiante
        ambiant = intensiteAmbiant * sphere.material.ambient_material;

        for(int i = 0; i < lights.size(); i++) {
            L = lights[i].pos - raySceneIntersection.raySphereIntersection.intersection; // Direction de la lumière : (PosLumière - PosSurface)

            // Normalisation de L, R et V obligatoire
            L.normalize();
            diffuse = intensiteDiffuse * sphere.material.diffuse_material * std::max((Vec3::dot(L, Normal)), 0.0f);

            V = ray.origin() - raySceneIntersection.raySphereIntersection.intersection; // 2ème rayon : (PosCamera(la position à partir de laquelle on voit) - PosSurface).
            V.normalize();

            R = 2 * (Vec3::dot(Normal, L) * Normal - L);
            R.normalize();

            speculaire = intensiteSpeculaire * sphere.material.specular_material * std::pow(std::max(0.0f, Vec3::dot(R, V)), sphere.material.shininess);

            intensiteDS += (diffuse + speculaire);       
            
            ComputeShadowsHardAndSoft_Sphere(lights, i, raySceneIntersection, estDansOmbre, L, marge, shadow, nbRayonsOmbres, tailleCarre, intensiteDS);
        }

        // Si il y a plusieurs lumières, l'éclairage est normale sans être trop éclairée.
        intensiteDS = intensiteDS / lights.size();

        int typeLumiere = 0;

        // Vérification du type des lumières
        if(lights[0].type == LightType_Quad) {
            typeLumiere = 1;
        } else {
            typeLumiere = 0;
        }


        if(typeLumiere == 0) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        } else if(typeLumiere == 1) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        }
                               
        // Clamp [0,1] --> permet de garder les couleurs dans l'intervalle [0,1]
        for(int i = 0; i < 3; i++) {
            clamp(color[i], 0., 1.);
        }
    } 

    void ComputePhongMaterial_Square(Vec3 &color, Square &square,RaySceneIntersection &raySceneIntersection, Ray &ray, Vec3 &ambiant, float intensiteAmbiant, Vec3 &diffuse, float intensiteDiffuse, Vec3 &speculaire, float intensiteSpeculaire, Vec3 &intensiteDS, Vec3 &L, Vec3 &R, Vec3 &V,  bool &estDansOmbre, float &shadow, int &nbRayonsOmbres, float &tailleCarre, float marge) {
        Vec3 Normal = raySceneIntersection.raySquareIntersection.normal;
        Normal.normalize();

        // Réfléxion ambiante
        ambiant = intensiteAmbiant * square.material.ambient_material;

        for(int i = 0; i < lights.size(); i++) {
            L = lights[i].pos - raySceneIntersection.raySquareIntersection.intersection; // Direction de la lumière : (PosLumière - PosSurface)
            // Normalisation de L, R et V obligatoire
            L.normalize();

            diffuse = intensiteDiffuse * square.material.diffuse_material * std::max((Vec3::dot(L, Normal)), 0.0f);

            V = ray.origin() - raySceneIntersection.raySquareIntersection.intersection; // 2ème rayon : (PosCamera(la position à partir de laquelle on voit) - PosSurface).
            V.normalize();

            R = 2 * (Vec3::dot(Normal, L) * Normal - L);
            R.normalize();

            speculaire = intensiteSpeculaire * square.material.specular_material * std::pow(std::max(0.0f, Vec3::dot(R, V)), square.material.shininess);

            intensiteDS += (diffuse + speculaire);  

            ComputeShadowsHardAndSoft_Square(lights, i, raySceneIntersection, estDansOmbre, L, marge, shadow, nbRayonsOmbres, tailleCarre, intensiteDS);
        }

        // Si il y a plusieurs lumières, l'éclairage est normale sans être trop éclairée.
        intensiteDS = intensiteDS / lights.size();

        int typeLumiere = 0;

        // Vérification du type des lumières
        if(lights[0].type == LightType_Quad) {
            typeLumiere = 1;
        } else {
            typeLumiere = 0;
        }


        if(typeLumiere == 0) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        } else if(typeLumiere == 1) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        }
            
        // Clamp [0,1] --> permet de garder les couleurs dans l'intervalle [0,1]
        for(int i = 0; i < 3; i++) {
            clamp(color[i], 0., 1.);
        }
    }     

    void ComputePhongMaterial_Mesh(Vec3 &color, Mesh &mesh, RaySceneIntersection &raySceneIntersection, Ray &ray, Vec3 &ambiant, float intensiteAmbiant, Vec3 &diffuse, float intensiteDiffuse, Vec3 &speculaire, float intensiteSpeculaire, Vec3 &intensiteDS, Vec3 &L, Vec3 &R, Vec3 &V,  bool &estDansOmbre, float &shadow, int &nbRayonsOmbres, float &tailleCarre, float marge) {
        Vec3 Normal = raySceneIntersection.rayMeshIntersection.normal;
        Normal.normalize();

        // Réfléxion ambiante
        ambiant = intensiteAmbiant * mesh.material.ambient_material;

        for(int i = 0; i < lights.size(); i++) {
            L = lights[i].pos - raySceneIntersection.rayMeshIntersection.intersection; // Direction de la lumière : (PosLumière - PosSurface)
            // Normalisation de L, R et V obligatoire
            L.normalize();

            diffuse = intensiteDiffuse * mesh.material.diffuse_material * std::max((Vec3::dot(L, Normal)), 0.0f);

            V = ray.origin() - raySceneIntersection.rayMeshIntersection.intersection; // 2ème rayon : (PosCamera(la position à partir de laquelle on voit) - PosSurface).
            V.normalize();

            R = 2 * (Vec3::dot(Normal, L) * Normal - L);
            R.normalize();

            speculaire = intensiteSpeculaire * mesh.material.specular_material * std::pow(std::max(0.0f, Vec3::dot(R, V)), mesh.material.shininess);

            intensiteDS += (diffuse + speculaire);  

            ComputeShadowsHardAndSoft_Mesh(lights, i, raySceneIntersection, estDansOmbre, L, marge, shadow, nbRayonsOmbres, tailleCarre, intensiteDS);
        }

        // Si il y a plusieurs lumières, l'éclairage est normale sans être trop éclairé.
        intensiteDS = intensiteDS / lights.size();

        int typeLumiere = 0;

        // Vérification du type des lumières
        if(lights[0].type == LightType_Quad) {
            typeLumiere = 1;
        } else {
            typeLumiere = 0;
        }


        if(typeLumiere == 0) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        } else if(typeLumiere == 1) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        }
            
        // Clamp [0,1] --> permet de garder les couleurs dans l'intervalle [0,1]
        for(int i = 0; i < 3; i++) {
            clamp(color[i], 0., 1.);
        }
    }  

    // Reflection Material
    void ComputeReflectionMaterial_Sphere(Vec3 &color, RaySceneIntersection &raySceneIntersection, Ray &ray, int &NRemainingBounces) {
        // Formule réfléxion : R_réfléchi = Incident - 2 * dot(N, Incident) * N
        Vec3 Normal = raySceneIntersection.raySphereIntersection.normal;
        Normal.normalize();

        // Vecteur incident
        Vec3 Incident = ray.direction();

        // Vecteur réfléchi
        Vec3 R = Incident - 2 * Vec3::dot(Normal, Incident)  * Normal;
        R.normalize();

        Ray reflection(raySceneIntersection.raySphereIntersection.intersection, R); 
        NRemainingBounces = NRemainingBounces - 1;
        color += rayTraceRecursive(reflection, NRemainingBounces);       
    }

    void ComputeReflectionMaterial_Square(Vec3 &color, RaySceneIntersection &raySceneIntersection, Ray &ray, int &NRemainingBounces) {
        // Formule réfléxion : R_réfléchi = Incident - 2 * dot(N, Incident) * N
        Vec3 Normal = raySceneIntersection.raySquareIntersection.normal;
        Normal.normalize();

        // Vecteur incident
        Vec3 Incident = ray.direction();

        // Vecteur réfléchi
        Vec3 R = Incident - 2 * Vec3::dot(Normal, Incident)  * Normal;
        R.normalize();

        Ray reflection(raySceneIntersection.raySquareIntersection.intersection, R); 
        NRemainingBounces = NRemainingBounces - 1;
        color += rayTraceRecursive(reflection, NRemainingBounces);       
    }

    void ComputeReflectionMaterial_Mesh(Vec3 &color, RaySceneIntersection &raySceneIntersection, Ray &ray, int &NRemainingBounces) {
        // Formule réfléxion : R_réfléchi = Incident - 2 * dot(N, Incident) * N
        Vec3 Normal = raySceneIntersection.rayMeshIntersection.normal;
        Normal.normalize();

        // Vecteur incident
        Vec3 Incident = ray.direction();

        // Vecteur réfléchi
        Vec3 R = Incident - 2 * Vec3::dot(Normal, Incident)  * Normal;
        R.normalize();

        Ray reflection(raySceneIntersection.rayMeshIntersection.intersection, R); 
        NRemainingBounces = NRemainingBounces - 1;
        color += rayTraceRecursive(reflection, NRemainingBounces);       
    }


    // Refraction Material
    void ComputeRefractionMaterial_Sphere(Vec3 &color, Sphere &sphere, RaySceneIntersection &raySceneIntersection, Ray &ray, int &NRemainingBounces) {
        Vec3 Normal = raySceneIntersection.raySphereIntersection.normal;
        Normal.normalize();

        Vec3 Incident = ray.direction();

        // n = n1/n2
        float n = sphere.material.transparency / sphere.material.index_medium;
        float C1 = Vec3::dot(Normal, Incident);
        float C2 = sqrt(1 - (n*n) * (1 - C1 * C1));

        Vec3 nI = n * Incident;
        float nC1C2 = n * C1 - C2;
        Vec3 nC1C2N = nC1C2 * Normal;

        Vec3 T = nI + nC1C2N; 

        Ray refraction(raySceneIntersection.raySphereIntersection.intersection, T); 
        NRemainingBounces = NRemainingBounces - 1;
        color += rayTraceRecursive(refraction, NRemainingBounces);     
    }

    void ComputeRefractionMaterial_Square(Vec3 &color, Square &square, RaySceneIntersection &raySceneIntersection, Ray &ray, int &NRemainingBounces) {
        Vec3 Normal = raySceneIntersection.raySquareIntersection.normal;
        Normal.normalize();

        Vec3 Incident = ray.direction();

        float n = square.material.transparency / square.material.index_medium;
        float C1 = Vec3::dot(Normal, Incident);
        float C2 = sqrt(1 - (n*n) * (1 - C1 * C1));

        // n = n1/n2
        Vec3 nI = n * Incident;
        float nC1C2 = n * C1 - C2;
        Vec3 nC1C2N = nC1C2 * Normal;

        Vec3 T = nI + nC1C2N; 

        Ray refraction(raySceneIntersection.raySquareIntersection.intersection, T); 
        NRemainingBounces = NRemainingBounces - 1;
        color += rayTraceRecursive(refraction, NRemainingBounces);    
    }

    void ComputeRefractionMaterial_Mesh(Vec3 &color, Mesh &mesh, RaySceneIntersection &raySceneIntersection, Ray &ray, int &NRemainingBounces) {
        Vec3 Normal = raySceneIntersection.rayMeshIntersection.normal;
        Normal.normalize();

        Vec3 Incident = ray.direction();

        // n = n1/n2
        float n = mesh.material.transparency / mesh.material.index_medium;
        float C1 = Vec3::dot(Normal, Incident);
        float C2 = sqrt(1 - (n*n) * (1 - C1 * C1));

        Vec3 nI = n * Incident;
        float nC1C2 = n * C1 - C2;
        Vec3 nC1C2N = nC1C2 * Normal;

        Vec3 T = nI + nC1C2N; 

        Ray refraction(raySceneIntersection.rayMeshIntersection.intersection, T); 
        NRemainingBounces = NRemainingBounces - 1;
        color += rayTraceRecursive(refraction, NRemainingBounces);   
    }


    // Blinn Phong Material
    void ComputeBlinnPhongMaterial_Sphere(Vec3 &color, Sphere &sphere,RaySceneIntersection &raySceneIntersection, Ray &ray, Vec3 &ambiant, float intensiteAmbiant, Vec3 &diffuse, float intensiteDiffuse, Vec3 &speculaire, float intensiteSpeculaire, Vec3 &intensiteDS, Vec3 &L, Vec3 &V, Vec3 &H, bool &estDansOmbre, float &shadow, int &nbRayonsOmbres, float &tailleCarre, float marge) {
        Vec3 Normal = raySceneIntersection.raySphereIntersection.normal;
        Normal.normalize();

        // Réfléxion ambiante
        ambiant = intensiteAmbiant * sphere.material.ambient_material;

        for(int i = 0; i < lights.size(); i++) {
            L = lights[i].pos - raySceneIntersection.raySphereIntersection.intersection; // Direction de la lumière : (PosLumière - PosSurface)

            // Normalisation de L, R et V obligatoire
            L.normalize();
            diffuse = intensiteDiffuse * sphere.material.diffuse_material * std::max((Vec3::dot(L, Normal)), 0.0f);

            V = ray.origin() - raySceneIntersection.raySphereIntersection.intersection; // 2ème rayon : (PosCamera(la position à partir de laquelle on voit) - PosSurface).
            V.normalize();

            H = V + L;
            H.normalize();

            speculaire = intensiteSpeculaire * sphere.material.specular_material * std::pow(std::max(0.0f, Vec3::dot(H, Normal)), sphere.material.shininess);    // ce n'est pas R.V mais H.N (Blinn-Phong)

            intensiteDS += (diffuse + speculaire);       

            ComputeShadowsHardAndSoft_Sphere(lights, i, raySceneIntersection, estDansOmbre, L, marge, shadow, nbRayonsOmbres, tailleCarre, intensiteDS);
        }

        // Si il y a plusieurs lumières, l'éclairage est normale sans être trop éclairée.
        intensiteDS = intensiteDS / lights.size();

        int typeLumiere = 0;

        // Vérification du type des lumières
        if(lights[0].type == LightType_Quad) {
            typeLumiere = 1;
        } else {
            typeLumiere = 0;
        }


        if(typeLumiere == 0) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        } else if(typeLumiere == 1) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        }
            
        // Clamp [0,1] --> permet de garder les couleurs dans l'intervalle [0,1]
        for(int i = 0; i < 3; i++) {
            clamp(color[i], 0., 1.);
        }
    } 

    void ComputeBlinnPhongMaterial_Square(Vec3 &color, Square &square,RaySceneIntersection &raySceneIntersection, Ray &ray, Vec3 &ambiant, float intensiteAmbiant, Vec3 &diffuse, float intensiteDiffuse, Vec3 &speculaire, float intensiteSpeculaire, Vec3 &intensiteDS, Vec3 &L, Vec3 &V, Vec3 &H, bool &estDansOmbre, float &shadow, int &nbRayonsOmbres, float &tailleCarre, float marge) {
        Vec3 Normal = raySceneIntersection.raySquareIntersection.normal;
        Normal.normalize();

        // Réfléxion ambiante
        ambiant = intensiteAmbiant * square.material.ambient_material;

        for(int i = 0; i < lights.size(); i++) {
            L = lights[i].pos - raySceneIntersection.raySquareIntersection.intersection; // Direction de la lumière : (PosLumière - PosSurface)
            // Normalisation de L, R et V obligatoire
            L.normalize();

            diffuse = intensiteDiffuse * square.material.diffuse_material * std::max((Vec3::dot(L, Normal)), 0.0f);

            V = ray.origin() - raySceneIntersection.raySquareIntersection.intersection; // 2ème rayon : (PosCamera(la position à partir de laquelle on voit) - PosSurface).
            V.normalize();

            H = V + L;
            H.normalize();

            speculaire = intensiteSpeculaire * square.material.specular_material * std::pow(std::max(0.0f, Vec3::dot(H, Normal)), square.material.shininess);

            intensiteDS += (diffuse + speculaire);  

            ComputeShadowsHardAndSoft_Square(lights, i, raySceneIntersection, estDansOmbre, L, marge, shadow, nbRayonsOmbres, tailleCarre, intensiteDS);
        }

        // Si il y a plusieurs lumières, l'éclairage est normale sans être trop éclairée.
        intensiteDS = intensiteDS / lights.size();

        int typeLumiere = 0;

        // Vérification du type des lumières
        if(lights[0].type == LightType_Quad) {
            typeLumiere = 1;
        } else {
            typeLumiere = 0;
        }


        if(typeLumiere == 0) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        } else if(typeLumiere == 1) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        }
            
        // Clamp [0,1] --> permet de garder les couleurs dans l'intervalle [0,1]
        for(int i = 0; i < 3; i++) {
            clamp(color[i], 0., 1.);
        }
    }     

    void ComputeBlinnPhongMaterial_Mesh(Vec3 &color, Mesh &mesh, RaySceneIntersection &raySceneIntersection, Ray &ray, Vec3 &ambiant, float intensiteAmbiant, Vec3 &diffuse, float intensiteDiffuse, Vec3 &speculaire, float intensiteSpeculaire, Vec3 &intensiteDS, Vec3 &L, Vec3 &V, Vec3 &H, bool &estDansOmbre, float &shadow, int &nbRayonsOmbres, float &tailleCarre, float marge) {
        Vec3 Normal = raySceneIntersection.rayMeshIntersection.normal;
        Normal.normalize();

        // Réfléxion ambiante
        ambiant = intensiteAmbiant * mesh.material.ambient_material;

        for(int i = 0; i < lights.size(); i++) {
            L = lights[i].pos - raySceneIntersection.rayMeshIntersection.intersection; // Direction de la lumière : (PosLumière - PosSurface)
            // Normalisation de L, R et V obligatoire
            L.normalize();

            diffuse = intensiteDiffuse * mesh.material.diffuse_material * std::max((Vec3::dot(L, Normal)), 0.0f);

            V = ray.origin() - raySceneIntersection.rayMeshIntersection.intersection; // 2ème rayon : (PosCamera(la position à partir de laquelle on voit) - PosSurface).
            V.normalize();

            H = V + L;
            H.normalize();

            speculaire = intensiteSpeculaire * mesh.material.specular_material * std::pow(std::max(0.0f, Vec3::dot(H, Normal)), mesh.material.shininess);

            intensiteDS += (diffuse + speculaire);  

            ComputeShadowsHardAndSoft_Mesh(lights, i, raySceneIntersection, estDansOmbre, L, marge, shadow, nbRayonsOmbres, tailleCarre, intensiteDS);
        }

        // Si il y a plusieurs lumières, l'éclairage est normale sans être trop éclairé.
        intensiteDS = intensiteDS / lights.size();

        int typeLumiere = 0;

        // Vérification du type des lumières
        if(lights[0].type == LightType_Quad) {
            typeLumiere = 1;
        } else {
            typeLumiere = 0;
        }


        if(typeLumiere == 0) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        } else if(typeLumiere == 1) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        }
            
        // Clamp [0,1] --> permet de garder les couleurs dans l'intervalle [0,1]
        for(int i = 0; i < 3; i++) {
            clamp(color[i], 0., 1.);
        }
    }  

    // Fresnel / Rim Material
    void ComputeFresnelRimMaterial_Sphere(Vec3 &color, Sphere &sphere,RaySceneIntersection &raySceneIntersection, Ray &ray, Vec3 &ambiant, float intensiteAmbiant, Vec3 &diffuse, float intensiteDiffuse, Vec3 &speculaire, float intensiteSpeculaire, Vec3 &intensiteDS, Vec3 &L, Vec3 &V, Vec3 &H, Vec3 &Rim, bool &estDansOmbre, float &shadow, int &nbRayonsOmbres, float &tailleCarre, float marge, float facteur_rim, Vec3 &fresnelColor, float intensite_rim) {
        Vec3 Normal = raySceneIntersection.raySphereIntersection.normal;
        Normal.normalize();

        // Réfléxion ambiante
        ambiant = intensiteAmbiant * sphere.material.ambient_material;

        for(int i = 0; i < lights.size(); i++) {
            L = lights[i].pos - raySceneIntersection.raySphereIntersection.intersection; // Direction de la lumière : (PosLumière - PosSurface)

            // Normalisation de L, R et V obligatoire
            L.normalize();
            diffuse = intensiteDiffuse * sphere.material.diffuse_material * std::max((Vec3::dot(L, Normal)), 0.0f);

            V = ray.origin() - raySceneIntersection.raySphereIntersection.intersection; // 2ème rayon : (PosCamera(la position à partir de laquelle on voit) - PosSurface).
            V.normalize();

            H = V + L;
            H.normalize();

            speculaire = intensiteSpeculaire * sphere.material.specular_material * std::pow(std::max(0.0f, Vec3::dot(H, Normal)), sphere.material.shininess);    // ce n'est pas R.V mais H.N (Blinn-Phong)
            

            // Calcul de l'effet de Rim
            // Calcul basique du facteur de Fresnel
            Rim[0] = std::pow(1.0 - std::max(0.0f, Vec3::dot(V, Normal)), facteur_rim);   // Fresnel (1 - v.n)
            Rim[1] = std::pow(1.0 - std::max(0.0f, Vec3::dot(V, Normal)), facteur_rim);   // Fresnel (1 - v.n)
            Rim[2] = std::pow(1.0 - std::max(0.0f, Vec3::dot(V, Normal)), facteur_rim);   // Fresnel (1 - v.n)

            // Pour contrôler l'effet de Fresnel
            Rim[0] = smoothstep(intensite_rim-0.01, intensite_rim+0.01, Rim[0]);
            Rim[1] = smoothstep(intensite_rim-0.01, intensite_rim+0.01, Rim[1]);
            Rim[2] = smoothstep(intensite_rim-0.01, intensite_rim+0.01, Rim[2]);

            // Pour avoir l'effet avec la couleur souhaitée
            Rim[0] *= fresnelColor[0];
            Rim[1] *= fresnelColor[1];
            Rim[2] *= fresnelColor[2];

            intensiteDS += (diffuse + speculaire + Rim);       

            ComputeShadowsHardAndSoft_Sphere(lights, i, raySceneIntersection, estDansOmbre, L, marge, shadow, nbRayonsOmbres, tailleCarre, intensiteDS);
        }

        // Si il y a plusieurs lumières, l'éclairage est normale sans être trop éclairée.
        intensiteDS = intensiteDS / lights.size();

        int typeLumiere = 0;

        // Vérification du type des lumières
        if(lights[0].type == LightType_Quad) {
            typeLumiere = 1;
        } else {
            typeLumiere = 0;
        }


        if(typeLumiere == 0) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        } else if(typeLumiere == 1) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        }
            
        // Clamp [0,1] --> permet de garder les couleurs dans l'intervalle [0,1]
        for(int i = 0; i < 3; i++) {
            clamp(color[i], 0., 1.);
        }
    } 

    void ComputeFresnelRimMaterial_Square(Vec3 &color, Square &square,RaySceneIntersection &raySceneIntersection, Ray &ray, Vec3 &ambiant, float intensiteAmbiant, Vec3 &diffuse, float intensiteDiffuse, Vec3 &speculaire, float intensiteSpeculaire, Vec3 &intensiteDS, Vec3 &L, Vec3 &V, Vec3 &H, Vec3 &Rim, bool &estDansOmbre, float &shadow, int &nbRayonsOmbres, float &tailleCarre, float marge, float facteur_rim, Vec3 &fresnelColor, float intensite_rim) {
        Vec3 Normal = raySceneIntersection.raySquareIntersection.normal;
        Normal.normalize();

        // Réfléxion ambiante
        ambiant = intensiteAmbiant * square.material.ambient_material;

        for(int i = 0; i < lights.size(); i++) {
            L = lights[i].pos - raySceneIntersection.raySquareIntersection.intersection; // Direction de la lumière : (PosLumière - PosSurface)
            // Normalisation de L, R et V obligatoire
            L.normalize();

            diffuse = intensiteDiffuse * square.material.diffuse_material * std::max((Vec3::dot(L, Normal)), 0.0f);

            V = ray.origin() - raySceneIntersection.raySquareIntersection.intersection; // 2ème rayon : (PosCamera(la position à partir de laquelle on voit) - PosSurface).
            V.normalize();

            H = V + L;
            H.normalize();

            speculaire = intensiteSpeculaire * square.material.specular_material * std::pow(std::max(0.0f, Vec3::dot(H, Normal)), square.material.shininess);

            // Calcul de l'effet de Rim
            // Calcul basique du facteur de Fresnel
            Rim[0] = std::pow(1.0 - std::max(0.0f, Vec3::dot(V, Normal)), facteur_rim);   // Fresnel (1 - v.n)
            Rim[1] = std::pow(1.0 - std::max(0.0f, Vec3::dot(V, Normal)), facteur_rim);   // Fresnel (1 - v.n)
            Rim[2] = std::pow(1.0 - std::max(0.0f, Vec3::dot(V, Normal)), facteur_rim);   // Fresnel (1 - v.n)

            // Pour contrôler l'effet de Fresnel
            Rim[0] = smoothstep(intensite_rim-0.01, intensite_rim+0.01, Rim[0]);
            Rim[1] = smoothstep(intensite_rim-0.01, intensite_rim+0.01, Rim[1]);
            Rim[2] = smoothstep(intensite_rim-0.01, intensite_rim+0.01, Rim[2]);

            // Pour avoir l'effet avec la couleur souhaitée
            Rim[0] *= fresnelColor[0];
            Rim[1] *= fresnelColor[1];
            Rim[2] *= fresnelColor[2];

            intensiteDS += (diffuse + speculaire + Rim);  

            ComputeShadowsHardAndSoft_Square(lights, i, raySceneIntersection, estDansOmbre, L, marge, shadow, nbRayonsOmbres, tailleCarre, intensiteDS);
        }

        // Si il y a plusieurs lumières, l'éclairage est normale sans être trop éclairée.
        intensiteDS = intensiteDS / lights.size();

        int typeLumiere = 0;

        // Vérification du type des lumières
        if(lights[0].type == LightType_Quad) {
            typeLumiere = 1;
        } else {
            typeLumiere = 0;
        }


        if(typeLumiere == 0) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        } else if(typeLumiere == 1) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        }
            
        // Clamp [0,1] --> permet de garder les couleurs dans l'intervalle [0,1]
        for(int i = 0; i < 3; i++) {
            clamp(color[i], 0., 1.);
        }
    }     

    void ComputeFresnelRimMaterial_Mesh(Vec3 &color, Mesh &mesh, RaySceneIntersection &raySceneIntersection, Ray &ray, Vec3 &ambiant, float intensiteAmbiant, Vec3 &diffuse, float intensiteDiffuse, Vec3 &speculaire, float intensiteSpeculaire, Vec3 &intensiteDS, Vec3 &L, Vec3 &V, Vec3 &H, Vec3 &Rim, bool &estDansOmbre, float &shadow, int &nbRayonsOmbres, float &tailleCarre, float marge, float facteur_rim, Vec3 &fresnelColor, float intensite_rim) {
        Vec3 Normal = raySceneIntersection.rayMeshIntersection.normal;
        Normal.normalize();

        // Réfléxion ambiante
        ambiant = intensiteAmbiant * mesh.material.ambient_material;

        for(int i = 0; i < lights.size(); i++) {
            L = lights[i].pos - raySceneIntersection.rayMeshIntersection.intersection; // Direction de la lumière : (PosLumière - PosSurface)
            // Normalisation de L, R et V obligatoire
            L.normalize();

            diffuse = intensiteDiffuse * mesh.material.diffuse_material * std::max((Vec3::dot(L, Normal)), 0.0f);

            V = ray.origin() - raySceneIntersection.rayMeshIntersection.intersection; // 2ème rayon : (PosCamera(la position à partir de laquelle on voit) - PosSurface).
            V.normalize();

            H = V + L;
            H.normalize();

            speculaire = intensiteSpeculaire * mesh.material.specular_material * std::pow(std::max(0.0f, Vec3::dot(H, Normal)), mesh.material.shininess);

            // Calcul de l'effet de Rim
            // Calcul basique du facteur de Fresnel
            Rim[0] = std::pow(1.0 - std::max(0.0f, Vec3::dot(V, Normal)), facteur_rim);   // Fresnel (1 - v.n)
            Rim[1] = std::pow(1.0 - std::max(0.0f, Vec3::dot(V, Normal)), facteur_rim);   // Fresnel (1 - v.n)
            Rim[2] = std::pow(1.0 - std::max(0.0f, Vec3::dot(V, Normal)), facteur_rim);   // Fresnel (1 - v.n)

            // Pour contrôler l'effet de Fresnel
            Rim[0] = smoothstep(intensite_rim-0.01, intensite_rim+0.01, Rim[0]);
            Rim[1] = smoothstep(intensite_rim-0.01, intensite_rim+0.01, Rim[1]);
            Rim[2] = smoothstep(intensite_rim-0.01, intensite_rim+0.01, Rim[2]);

            // Pour avoir l'effet avec la couleur souhaitée
            Rim[0] *= fresnelColor[0];
            Rim[1] *= fresnelColor[1];
            Rim[2] *= fresnelColor[2];

            intensiteDS += (diffuse + speculaire + Rim);   

            ComputeShadowsHardAndSoft_Mesh(lights, i, raySceneIntersection, estDansOmbre, L, marge, shadow, nbRayonsOmbres, tailleCarre, intensiteDS);
        }

        // Si il y a plusieurs lumières, l'éclairage est normale sans être trop éclairé.
        intensiteDS = intensiteDS / lights.size();

        int typeLumiere = 0;

        // Vérification du type des lumières
        if(lights[0].type == LightType_Quad) {
            typeLumiere = 1;
        } else {
            typeLumiere = 0;
        }


        if(typeLumiere == 0) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        } else if(typeLumiere == 1) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        }
            
        // Clamp [0,1] --> permet de garder les couleurs dans l'intervalle [0,1]
        for(int i = 0; i < 3; i++) {
            clamp(color[i], 0., 1.);
        }
    }  

    // Toon
    void ComputeToonMaterial_Sphere(Vec3 &color, Sphere &sphere,RaySceneIntersection &raySceneIntersection, Ray &ray, Vec3 &ambiant, float intensiteAmbiant, Vec3 &diffuse, float intensiteDiffuse, Vec3 &speculaire, float intensiteSpeculaire, Vec3 &intensiteDS, Vec3 &L, Vec3 &V, Vec3 &H, Vec3 &Rim, bool &estDansOmbre, float &shadow, int &nbRayonsOmbres, float &tailleCarre, float marge, float facteur_rim, Vec3 &fresnelColor, float intensite_rim, float nSteps) {
        Vec3 Normal = raySceneIntersection.raySphereIntersection.normal;
        Normal.normalize();

        // Réfléxion ambiante
        ambiant = intensiteAmbiant * sphere.material.ambient_material;

        for(int i = 0; i < lights.size(); i++) {
            L = lights[i].pos - raySceneIntersection.raySphereIntersection.intersection; // Direction de la lumière : (PosLumière - PosSurface)

            // Normalisation de L, R et V obligatoire
            L.normalize();


            double diff = std::max(Vec3::dot(L, Normal), 0.0f);
            float step = sqrt(diff) * nSteps;
            step = (floor(step) + smoothstep(0.48, 0.52, fract(step))) / nSteps;
            diffuse = step * intensiteDiffuse * sphere.material.diffuse_material;

            V = ray.origin() - raySceneIntersection.raySphereIntersection.intersection; // 2ème rayon : (PosCamera(la position à partir de laquelle on voit) - PosSurface).
            V.normalize();

            H = V + L;
            H.normalize();

            speculaire = intensiteSpeculaire * sphere.material.specular_material * std::pow(std::max(0.0f, Vec3::dot(H, Normal)), sphere.material.shininess);

            // Calcul de l'effet de Rim
            // Calcul basique du facteur de Fresnel
            Rim[0] = std::pow(1.0 - std::max(0.0f, Vec3::dot(V, Normal)), facteur_rim);   // Fresnel (1 - v.n)
            Rim[1] = std::pow(1.0 - std::max(0.0f, Vec3::dot(V, Normal)), facteur_rim);   // Fresnel (1 - v.n)
            Rim[2] = std::pow(1.0 - std::max(0.0f, Vec3::dot(V, Normal)), facteur_rim);   // Fresnel (1 - v.n)

            // Pour contrôler l'effet de Fresnel
            Rim[0] = smoothstep(intensite_rim-0.01, intensite_rim+0.01, Rim[0]);
            Rim[1] = smoothstep(intensite_rim-0.01, intensite_rim+0.01, Rim[1]);
            Rim[2] = smoothstep(intensite_rim-0.01, intensite_rim+0.01, Rim[2]);

            // Pour avoir l'effet avec la couleur souhaitée
            Rim[0] *= fresnelColor[0];
            Rim[1] *= fresnelColor[1];
            Rim[2] *= fresnelColor[2];

            intensiteDS += (diffuse + speculaire + Rim);       

            ComputeShadowsHardAndSoft_Sphere(lights, i, raySceneIntersection, estDansOmbre, L, marge, shadow, nbRayonsOmbres, tailleCarre, intensiteDS);
        }

        // Si il y a plusieurs lumières, l'éclairage est normale sans être trop éclairée.
        intensiteDS = intensiteDS / lights.size();

        int typeLumiere = 0;

        // Vérification du type des lumières
        if(lights[0].type == LightType_Quad) {
            typeLumiere = 1;
        } else {
            typeLumiere = 0;
        }


        if(typeLumiere == 0) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        } else if(typeLumiere == 1) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        }
            
        // Clamp [0,1] --> permet de garder les couleurs dans l'intervalle [0,1]
        for(int i = 0; i < 3; i++) {
            clamp(color[i], 0., 1.);
        }
    } 

    void ComputeToonMaterial_Square(Vec3 &color, Square &square,RaySceneIntersection &raySceneIntersection, Ray &ray, Vec3 &ambiant, float intensiteAmbiant, Vec3 &diffuse, float intensiteDiffuse, Vec3 &speculaire, float intensiteSpeculaire, Vec3 &intensiteDS, Vec3 &L, Vec3 &V, Vec3 &H, Vec3 &Rim, bool &estDansOmbre, float &shadow, int &nbRayonsOmbres, float &tailleCarre, float marge, float facteur_rim, Vec3 &fresnelColor, float intensite_rim, float nSteps) {
        Vec3 Normal = raySceneIntersection.raySquareIntersection.normal;
        Normal.normalize();

        // Réfléxion ambiante
        ambiant = intensiteAmbiant * square.material.ambient_material;

        for(int i = 0; i < lights.size(); i++) {
            L = lights[i].pos - raySceneIntersection.raySquareIntersection.intersection; // Direction de la lumière : (PosLumière - PosSurface)
            // Normalisation de L, R et V obligatoire
            L.normalize();

            double diff = std::max(Vec3::dot(L, Normal), 0.0f);
            float step = sqrt(diff) * nSteps;
            step = (floor(step) + smoothstep(0.48, 0.52, fract(step))) / nSteps;
            diffuse = step * intensiteDiffuse * square.material.diffuse_material;

            V = ray.origin() - raySceneIntersection.raySquareIntersection.intersection; // 2ème rayon : (PosCamera(la position à partir de laquelle on voit) - PosSurface).
            V.normalize();

            H = V + L;
            H.normalize();

            speculaire = intensiteSpeculaire * square.material.specular_material * std::pow(std::max(0.0f, Vec3::dot(H, Normal)), square.material.shininess);

            // Calcul de l'effet de Rim
            // Calcul basique du facteur de Fresnel
            Rim[0] = std::pow(1.0 - std::max(0.0f, Vec3::dot(V, Normal)), facteur_rim);   // Fresnel (1 - v.n)
            Rim[1] = std::pow(1.0 - std::max(0.0f, Vec3::dot(V, Normal)), facteur_rim);   // Fresnel (1 - v.n)
            Rim[2] = std::pow(1.0 - std::max(0.0f, Vec3::dot(V, Normal)), facteur_rim);   // Fresnel (1 - v.n)

            // Pour contrôler l'effet de Fresnel
            Rim[0] = smoothstep(intensite_rim-0.01, intensite_rim+0.01, Rim[0]);
            Rim[1] = smoothstep(intensite_rim-0.01, intensite_rim+0.01, Rim[1]);
            Rim[2] = smoothstep(intensite_rim-0.01, intensite_rim+0.01, Rim[2]);

            // Pour avoir l'effet avec la couleur souhaitée
            Rim[0] *= fresnelColor[0];
            Rim[1] *= fresnelColor[1];
            Rim[2] *= fresnelColor[2];

            intensiteDS += (diffuse + speculaire + Rim);  

            ComputeShadowsHardAndSoft_Square(lights, i, raySceneIntersection, estDansOmbre, L, marge, shadow, nbRayonsOmbres, tailleCarre, intensiteDS);
        }

        // Si il y a plusieurs lumières, l'éclairage est normale sans être trop éclairée.
        intensiteDS = intensiteDS / lights.size();

        int typeLumiere = 0;

        // Vérification du type des lumières
        if(lights[0].type == LightType_Quad) {
            typeLumiere = 1;
        } else {
            typeLumiere = 0;
        }


        if(typeLumiere == 0) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        } else if(typeLumiere == 1) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        }
            
        // Clamp [0,1] --> permet de garder les couleurs dans l'intervalle [0,1]
        for(int i = 0; i < 3; i++) {
            clamp(color[i], 0., 1.);
        }
    }     

    void ComputeToonMaterial_Mesh(Vec3 &color, Mesh &mesh, RaySceneIntersection &raySceneIntersection, Ray &ray, Vec3 &ambiant, float intensiteAmbiant, Vec3 &diffuse, float intensiteDiffuse, Vec3 &speculaire, float intensiteSpeculaire, Vec3 &intensiteDS, Vec3 &L, Vec3 &V, Vec3 &H, Vec3 &Rim, bool &estDansOmbre, float &shadow, int &nbRayonsOmbres, float &tailleCarre, float marge, float facteur_rim, Vec3 &fresnelColor, float intensite_rim, float nSteps) {
        Vec3 Normal = raySceneIntersection.rayMeshIntersection.normal;
        Normal.normalize();

        // Réfléxion ambiante
        ambiant = intensiteAmbiant * mesh.material.ambient_material;

        for(int i = 0; i < lights.size(); i++) {
            L = lights[i].pos - raySceneIntersection.rayMeshIntersection.intersection; // Direction de la lumière : (PosLumière - PosSurface)
            // Normalisation de L, R et V obligatoire
            L.normalize();

            double diff = std::max(Vec3::dot(L, Normal), 0.0f);
            float step = sqrt(diff) * nSteps;
            step = (floor(step) + smoothstep(0.48, 0.52, fract(step))) / nSteps;
            diffuse = step * intensiteDiffuse * mesh.material.diffuse_material;

            V = ray.origin() - raySceneIntersection.rayMeshIntersection.intersection; // 2ème rayon : (PosCamera(la position à partir de laquelle on voit) - PosSurface).
            V.normalize();

            H = V + L;
            H.normalize();

            speculaire = intensiteSpeculaire * mesh.material.specular_material * std::pow(std::max(0.0f, Vec3::dot(H, Normal)), mesh.material.shininess);

            // Calcul de l'effet de Rim
            // Calcul basique du facteur de Fresnel
            Rim[0] = std::pow(1.0 - std::max(0.0f, Vec3::dot(V, Normal)), facteur_rim);   // Fresnel (1 - v.n)
            Rim[1] = std::pow(1.0 - std::max(0.0f, Vec3::dot(V, Normal)), facteur_rim);   // Fresnel (1 - v.n)
            Rim[2] = std::pow(1.0 - std::max(0.0f, Vec3::dot(V, Normal)), facteur_rim);   // Fresnel (1 - v.n)

            // Pour contrôler l'effet de Fresnel
            Rim[0] = smoothstep(intensite_rim-0.01, intensite_rim+0.01, Rim[0]);
            Rim[1] = smoothstep(intensite_rim-0.01, intensite_rim+0.01, Rim[1]);
            Rim[2] = smoothstep(intensite_rim-0.01, intensite_rim+0.01, Rim[2]);

            // Pour avoir l'effet avec la couleur souhaitée
            Rim[0] *= fresnelColor[0];
            Rim[1] *= fresnelColor[1];
            Rim[2] *= fresnelColor[2];

            intensiteDS += (diffuse + speculaire + Rim);   

            ComputeShadowsHardAndSoft_Mesh(lights, i, raySceneIntersection, estDansOmbre, L, marge, shadow, nbRayonsOmbres, tailleCarre, intensiteDS);
        }

        // Si il y a plusieurs lumières, l'éclairage est normale sans être trop éclairé.
        intensiteDS = intensiteDS / lights.size();

        int typeLumiere = 0;

        // Vérification du type des lumières
        if(lights[0].type == LightType_Quad) {
            typeLumiere = 1;
        } else {
            typeLumiere = 0;
        }


        if(typeLumiere == 0) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        } else if(typeLumiere == 1) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        }
            
        // Clamp [0,1] --> permet de garder les couleurs dans l'intervalle [0,1]
        for(int i = 0; i < 3; i++) {
            clamp(color[i], 0., 1.);
        }
    }  


    void ComputePBRMaterial_Sphere(Vec3 &color, Sphere &sphere,RaySceneIntersection &raySceneIntersection, Ray &ray, Vec3 &ambiant, float intensiteAmbiant, Vec3 &diffuse, float intensiteDiffuse, Vec3 &speculaire, float intensiteSpeculaire, Vec3 &intensiteDS, Vec3 &L, Vec3 &V, Vec3 &H, bool &estDansOmbre, float &shadow, int &nbRayonsOmbres, float &tailleCarre, float marge, double roughness, double metallic) {
        Vec3 Normal = raySceneIntersection.raySphereIntersection.normal;
        Normal.normalize();

        ambiant = intensiteAmbiant * sphere.material.ambient_material;

        for(int i = 0; i < lights.size(); i++) {
            L = lights[i].pos - raySceneIntersection.raySphereIntersection.intersection; 

            L.normalize();

            // Lambertian Diffuse
            diffuse = intensiteDiffuse * sphere.material.diffuse_material;
            diffuse[0] /= M_PI;
            diffuse[1] /= M_PI;
            diffuse[2] /= M_PI;

            V = ray.origin() - raySceneIntersection.raySphereIntersection.intersection; 

            H = V + L;
            H.normalize();

            double d, f, g;
            d = DistributionGGX(Normal, H, roughness);
            g = GeometrySmith(Normal, V, L, roughness);
            f = F(sphere.material.transparency, sphere.material.index_medium, 5., V, H);
            
            speculaire[0] = (d * f * g) / (4 * std::max(Vec3::dot(Normal, V), 0.0f) * std::max(Vec3::dot(L, Normal), 0.0f) + 0.0001);   // + 0.0001 pour éviter les divisions par zéro
            speculaire[1] = (d * f * g) / (4 * std::max(Vec3::dot(Normal, V), 0.0f) * std::max(Vec3::dot(L, Normal), 0.0f) + 0.0001);
            speculaire[2] = (d * f * g) / (4 * std::max(Vec3::dot(Normal, V), 0.0f) * std::max(Vec3::dot(L, Normal), 0.0f) + 0.0001);

            double kS, kD;
            kS = f;
            kD = 1. - kS;
            kD *= 1. - metallic;    

            intensiteDS[0] += (kD * diffuse[0] + speculaire[0]) * std::max(Vec3::dot(Normal, L), 0.0f);
            intensiteDS[1] += (kD * diffuse[1] + speculaire[1]) * std::max(Vec3::dot(Normal, L), 0.0f);
            intensiteDS[2] += (kD * diffuse[2] + speculaire[2]) * std::max(Vec3::dot(Normal, L), 0.0f);

            ComputeShadowsHardAndSoft_Sphere(lights, i, raySceneIntersection, estDansOmbre, L, marge, shadow, nbRayonsOmbres, tailleCarre, intensiteDS);
        }

        intensiteDS = intensiteDS / lights.size();

        int typeLumiere = 0;

        if(lights[0].type == LightType_Quad) {
            typeLumiere = 1;
        } else {
            typeLumiere = 0;
        }


        if(typeLumiere == 0) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        } else if(typeLumiere == 1) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        }
            
        // Clamp [0,1] --> permet de garder les couleurs dans l'intervalle [0,1]
        for(int i = 0; i < 3; i++) {
            clamp(color[i], 0., 1.);
        }
    } 

    void ComputePBRMaterial_Square(Vec3 &color, Square &square,RaySceneIntersection &raySceneIntersection, Ray &ray, Vec3 &ambiant, float intensiteAmbiant, Vec3 &diffuse, float intensiteDiffuse, Vec3 &speculaire, float intensiteSpeculaire, Vec3 &intensiteDS, Vec3 &L, Vec3 &V, Vec3 &H, bool &estDansOmbre, float &shadow, int &nbRayonsOmbres, float &tailleCarre, float marge, double roughness, double metallic) {
        Vec3 Normal = raySceneIntersection.raySquareIntersection.normal;
        Normal.normalize();

        ambiant = intensiteAmbiant * square.material.ambient_material;

        for(int i = 0; i < lights.size(); i++) {
            L = lights[i].pos - raySceneIntersection.raySquareIntersection.intersection; 
            L.normalize();

            // Lambertian Diffuse
            diffuse = intensiteDiffuse * square.material.diffuse_material;
            diffuse[0] /= M_PI;
            diffuse[1] /= M_PI;
            diffuse[2] /= M_PI;

            V = ray.origin() - raySceneIntersection.raySquareIntersection.intersection; 
            V.normalize();

            H = V + L;
            H.normalize();

            double d, f, g;
            d = DistributionGGX(Normal, H, roughness);
            g = GeometrySmith(Normal, V, L, roughness);
            f = F(square.material.transparency, square.material.index_medium, 5., V, H);
            
            speculaire[0] = (d * f * g) / (4 * std::max(Vec3::dot(Normal, V), 0.0f) * std::max(Vec3::dot(L, Normal), 0.0f) + 0.0001);   // + 0.0001 pour éviter les divisions par zéro
            speculaire[1] = (d * f * g) / (4 * std::max(Vec3::dot(Normal, V), 0.0f) * std::max(Vec3::dot(L, Normal), 0.0f) + 0.0001);
            speculaire[2] = (d * f * g) / (4 * std::max(Vec3::dot(Normal, V), 0.0f) * std::max(Vec3::dot(L, Normal), 0.0f) + 0.0001);

            double kS, kD;
            kS = f;
            kD = 1. - kS;
            kD *= 1. - metallic;    

            intensiteDS[0] += (kD * diffuse[0] + speculaire[0]) * std::max(Vec3::dot(Normal, L), 0.0f);
            intensiteDS[1] += (kD * diffuse[1] + speculaire[1]) * std::max(Vec3::dot(Normal, L), 0.0f);
            intensiteDS[2] += (kD * diffuse[2] + speculaire[2]) * std::max(Vec3::dot(Normal, L), 0.0f);

            ComputeShadowsHardAndSoft_Square(lights, i, raySceneIntersection, estDansOmbre, L, marge, shadow, nbRayonsOmbres, tailleCarre, intensiteDS);
        }

        intensiteDS = intensiteDS / lights.size();

        int typeLumiere = 0;

        if(lights[0].type == LightType_Quad) {
            typeLumiere = 1;
        } else {
            typeLumiere = 0;
        }


        if(typeLumiere == 0) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        } else if(typeLumiere == 1) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        }
            
        // Clamp [0,1] --> permet de garder les couleurs dans l'intervalle [0,1]
        for(int i = 0; i < 3; i++) {
            clamp(color[i], 0., 1.);
        }
    }     

    void ComputePBRMaterial_Mesh(Vec3 &color, Mesh &mesh, RaySceneIntersection &raySceneIntersection, Ray &ray, Vec3 &ambiant, float intensiteAmbiant, Vec3 &diffuse, float intensiteDiffuse, Vec3 &speculaire, float intensiteSpeculaire, Vec3 &intensiteDS, Vec3 &L, Vec3 &V, Vec3 &H, bool &estDansOmbre, float &shadow, int &nbRayonsOmbres, float &tailleCarre, float marge, double roughness, double metallic) {
        Vec3 Normal = raySceneIntersection.rayMeshIntersection.normal;
        Normal.normalize();

        ambiant = intensiteAmbiant * mesh.material.ambient_material;

        for(int i = 0; i < lights.size(); i++) {
            L = lights[i].pos - raySceneIntersection.rayMeshIntersection.intersection; 
            L.normalize();

            // Lambertian Diffuse
            diffuse = intensiteDiffuse * mesh.material.diffuse_material;
            diffuse[0] /= M_PI;
            diffuse[1] /= M_PI;
            diffuse[2] /= M_PI;

            V = ray.origin() - raySceneIntersection.rayMeshIntersection.intersection;
            V.normalize();

            H = V + L;
            H.normalize();

            double d, f, g;
            d = DistributionGGX(Normal, H, roughness);
            g = GeometrySmith(Normal, V, L, roughness);
            f = F(mesh.material.transparency, mesh.material.index_medium, 5., V, H);
            
            speculaire[0] = (d * f * g) / (4 * std::max(Vec3::dot(Normal, V), 0.0f) * std::max(Vec3::dot(L, Normal), 0.0f) + 0.0001);   // + 0.0001 pour éviter les divisions par zéro
            speculaire[1] = (d * f * g) / (4 * std::max(Vec3::dot(Normal, V), 0.0f) * std::max(Vec3::dot(L, Normal), 0.0f) + 0.0001);
            speculaire[2] = (d * f * g) / (4 * std::max(Vec3::dot(Normal, V), 0.0f) * std::max(Vec3::dot(L, Normal), 0.0f) + 0.0001);

            double kS, kD;
            kS = f;
            kD = 1. - kS;
            kD *= 1. - metallic;    

            intensiteDS[0] += (kD * diffuse[0] + speculaire[0]) * std::max(Vec3::dot(Normal, L), 0.0f);
            intensiteDS[1] += (kD * diffuse[1] + speculaire[1]) * std::max(Vec3::dot(Normal, L), 0.0f);
            intensiteDS[2] += (kD * diffuse[2] + speculaire[2]) * std::max(Vec3::dot(Normal, L), 0.0f);

            ComputeShadowsHardAndSoft_Mesh(lights, i, raySceneIntersection, estDansOmbre, L, marge, shadow, nbRayonsOmbres, tailleCarre, intensiteDS);
        }

        intensiteDS = intensiteDS / lights.size();

        int typeLumiere = 0;

        if(lights[0].type == LightType_Quad) {
            typeLumiere = 1;
        } else {
            typeLumiere = 0;
        }


        if(typeLumiere == 0) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        } else if(typeLumiere == 1) {
            if(estDansOmbre == true) {
                color = Vec3(0., 0., 0.);
            } else {
                color = (ambiant + intensiteDS);
            }
        }
            
        // Clamp [0,1] --> permet de garder les couleurs dans l'intervalle [0,1]
        for(int i = 0; i < 3; i++) {
            clamp(color[i], 0., 1.);
        }
    }  

    // Texture
    void ComputeTextureMaterial_Sphere(RaySceneIntersection &raySceneIntersection, Vec3 &color, Sphere &sphere) {
        Texture tex(raySceneIntersection.raySphereIntersection.u, raySceneIntersection.raySphereIntersection.v);
        if(sphere.material.textype == Texture_Color) {
            tex.setConstantColor(Vec3(0., 1., 0.));
        } else if(sphere.material.textype == Texture_Checkered) {
            tex.setCheckeredTexture(sphere.material.nbRepetitions);
        } else if(sphere.material.textype == Texture_WhiteNoise) {
            tex.setWhiteNoiseTexture();
        } else if(sphere.material.textype == Texture_File) {
            tex.setTextureFromFilePPM("img/sphereTextures/s2.ppm");
        }
        color = tex.color;
    } 

    void ComputeTextureMaterial_Square(RaySceneIntersection &raySceneIntersection, Vec3 &color, Square &square) {
        Texture tex(raySceneIntersection.raySquareIntersection.u, raySceneIntersection.raySquareIntersection.v);
        if(square.material.textype == Texture_Color) {
            tex.setConstantColor(Vec3(0., 1., 0.));
        } else if(square.material.textype == Texture_Checkered) {
            tex.setCheckeredTexture(square.material.nbRepetitions);
        } else if(square.material.textype == Texture_WhiteNoise) {
            tex.setWhiteNoiseTexture();
        } else if(square.material.textype == Texture_File) {
            tex.setTextureFromFilePPM("img/sphereTextures/s2.ppm");
        }
        color = tex.color;
    }     



    Vec3 rayTraceRecursive( Ray ray , int NRemainingBounces ) {
        //TODO RaySceneIntersection raySceneIntersection = computeIntersection(ray);
        RaySceneIntersection raySceneIntersection = computeIntersection(ray);
        Vec3 color = Vec3(0., 0., 0.);
        Vec3 intensiteDS = Vec3(0., 0., 0.);

        // Ambiant
        float intensiteAmbiant = 1.0f;
        Vec3 ambiant;

        // Diffuse
        float intensiteDiffuse = 1.0f;
        Vec3 diffuse;

        // Spéculaire
        float intensiteSpeculaire = 0.05f;
        Vec3 speculaire;

        // Ombres
        bool estDansOmbre = false;
        float shadow;

        // Nb d'échantillons --> ombres douces
        int nbRayonsOmbres = 1;

        // Carré de lumières
        float tailleCarre = 2.0f;

        float marge = 0.001;
        float marge_Mesh = 0.001;

        Vec3 L, R, V;

        Vec3 H; // Modèle d'éclairage de Blinn-Phong

        Vec3 Rim, fresnelColor = Vec3(1., 1., 1.);   // Modèle d'éclairage de Rim
        float rimFactor = 5;
        float rimIntensity = 0.25;

        // Toon
        float nSteps = 7.;

        // PBR
        float roughness = 1.f;
        float metallic = 1.f;

        bool isInterExist = raySceneIntersection.intersectionExists;
        int typeOfIntersectedObject = raySceneIntersection.typeOfIntersectedObject;

        if(isInterExist == true) {
            if(typeOfIntersectedObject == 0) {
                // Basique
                if(spheres[raySceneIntersection.objectIndex].material.type == Material_Basic) {
                    ComputeBasicMaterial_Sphere(color, spheres[raySceneIntersection.objectIndex]);                
                }
                // Phong
                if(spheres[raySceneIntersection.objectIndex].material.type == Material_Phong) {
                    ComputePhongMaterial_Sphere(color, spheres[raySceneIntersection.objectIndex], raySceneIntersection, ray, ambiant, intensiteAmbiant, diffuse, intensiteDiffuse, speculaire, intensiteSpeculaire, intensiteDS, L, R, V, estDansOmbre, shadow, nbRayonsOmbres, tailleCarre, marge);
                }
                // Réflexion
                if(spheres[raySceneIntersection.objectIndex].material.type == Material_Mirror && NRemainingBounces > 0) {
                    ComputeReflectionMaterial_Sphere(color, raySceneIntersection, ray, NRemainingBounces);          
                }
                // Réfraction
                if(spheres[raySceneIntersection.objectIndex].material.type == Material_Glass && NRemainingBounces > 0) {
                    ComputeRefractionMaterial_Sphere(color, spheres[raySceneIntersection.objectIndex], raySceneIntersection, ray, NRemainingBounces);                                      
                }
                // Blinn-Phong
                if(spheres[raySceneIntersection.objectIndex].material.type == Material_Blinn_Phong) {
                    ComputeBlinnPhongMaterial_Sphere(color, spheres[raySceneIntersection.objectIndex], raySceneIntersection, ray, ambiant, intensiteAmbiant, diffuse, intensiteDiffuse, speculaire, intensiteSpeculaire, intensiteDS, L, V, H, estDansOmbre, shadow, nbRayonsOmbres, tailleCarre, marge);
                }
                // Fresnel / Rim
                if(spheres[raySceneIntersection.objectIndex].material.type == Material_FresnelRim) {
                    ComputeFresnelRimMaterial_Sphere(color, spheres[raySceneIntersection.objectIndex], raySceneIntersection, ray, ambiant, intensiteAmbiant, diffuse, intensiteDiffuse, speculaire, intensiteSpeculaire, intensiteDS, L, V, H, Rim, estDansOmbre, shadow, nbRayonsOmbres, tailleCarre, marge, rimFactor, fresnelColor, rimIntensity);
                }
                // Toon
                if(spheres[raySceneIntersection.objectIndex].material.type == Material_Toon) {
                    ComputeToonMaterial_Sphere(color, spheres[raySceneIntersection.objectIndex], raySceneIntersection, ray, ambiant, intensiteAmbiant, diffuse, intensiteDiffuse, speculaire, intensiteSpeculaire, intensiteDS, L, V, H, Rim, estDansOmbre, shadow, nbRayonsOmbres, tailleCarre, marge, rimFactor, fresnelColor, rimIntensity, nSteps);
                }
                // PBR
                if(spheres[raySceneIntersection.objectIndex].material.type == Material_PBR) {
                    ComputePBRMaterial_Sphere(color, spheres[raySceneIntersection.objectIndex], raySceneIntersection, ray, ambiant, intensiteAmbiant, diffuse, intensiteDiffuse, speculaire, intensiteSpeculaire, intensiteDS, L, V, H, estDansOmbre, shadow, nbRayonsOmbres, tailleCarre, marge, roughness, metallic);
                }
                // Texture
                if(spheres[raySceneIntersection.objectIndex].material.type == Material_Texture) {
                    ComputeTextureMaterial_Sphere(raySceneIntersection, color, spheres[raySceneIntersection.objectIndex]);
                }
            }

            if(typeOfIntersectedObject == 1) {
                // Basique
                if(squares[raySceneIntersection.objectIndex].material.type == Material_Basic) {
                    ComputeBasicMaterial_Square(color, squares[raySceneIntersection.objectIndex]);                
                }
                // Phong
                if(squares[raySceneIntersection.objectIndex].material.type == Material_Phong) {
                    ComputePhongMaterial_Square(color, squares[raySceneIntersection.objectIndex], raySceneIntersection, ray, ambiant, intensiteAmbiant, diffuse, intensiteDiffuse, speculaire, intensiteSpeculaire, intensiteDS, L, R, V, estDansOmbre, shadow, nbRayonsOmbres, tailleCarre, marge);
                }
                // Réflexion
                if(squares[raySceneIntersection.objectIndex].material.type == Material_Mirror && NRemainingBounces > 0) {
                    ComputeReflectionMaterial_Square(color, raySceneIntersection, ray, NRemainingBounces);            
                }
                // Réfraction
                if(squares[raySceneIntersection.objectIndex].material.type == Material_Glass && NRemainingBounces > 0) {
                    ComputeRefractionMaterial_Square(color, squares[raySceneIntersection.objectIndex], raySceneIntersection, ray, NRemainingBounces);                                                             
                }
                // Blinn-Phong
                if(squares[raySceneIntersection.objectIndex].material.type == Material_Blinn_Phong) {
                    ComputeBlinnPhongMaterial_Square(color, squares[raySceneIntersection.objectIndex], raySceneIntersection, ray, ambiant, intensiteAmbiant, diffuse, intensiteDiffuse, speculaire, intensiteSpeculaire, intensiteDS, L, V, H, estDansOmbre, shadow, nbRayonsOmbres, tailleCarre, marge);
                }
                // Fresnel / Rim
                if(squares[raySceneIntersection.objectIndex].material.type == Material_FresnelRim) {
                    ComputeFresnelRimMaterial_Square(color, squares[raySceneIntersection.objectIndex], raySceneIntersection, ray, ambiant, intensiteAmbiant, diffuse, intensiteDiffuse, speculaire, intensiteSpeculaire, intensiteDS, L, V, H, Rim, estDansOmbre, shadow, nbRayonsOmbres, tailleCarre, marge, rimFactor, fresnelColor, rimIntensity);
                }
                // Toon
                if(squares[raySceneIntersection.objectIndex].material.type == Material_Toon) {
                    ComputeToonMaterial_Square(color, squares[raySceneIntersection.objectIndex], raySceneIntersection, ray, ambiant, intensiteAmbiant, diffuse, intensiteDiffuse, speculaire, intensiteSpeculaire, intensiteDS, L, V, H, Rim, estDansOmbre, shadow, nbRayonsOmbres, tailleCarre, marge, rimFactor, fresnelColor, rimIntensity, nSteps);
                }
                // PBR
                if(squares[raySceneIntersection.objectIndex].material.type == Material_PBR) {
                    ComputePBRMaterial_Square(color, squares[raySceneIntersection.objectIndex], raySceneIntersection, ray, ambiant, intensiteAmbiant, diffuse, intensiteDiffuse, speculaire, intensiteSpeculaire, intensiteDS, L, V, H, estDansOmbre, shadow, nbRayonsOmbres, tailleCarre, marge, roughness, metallic);
                }
                // Texture
                if(squares[raySceneIntersection.objectIndex].material.type == Material_Texture) {
                    ComputeTextureMaterial_Square(raySceneIntersection, color, squares[raySceneIntersection.objectIndex]);
                }
            }
            if(typeOfIntersectedObject == 2) {
                // Basique
                if(meshes[raySceneIntersection.objectIndex].material.type == Material_Basic) {
                    ComputeBasicMaterial_Mesh(color, meshes[raySceneIntersection.objectIndex]);                
                }
                // Phong
                if(meshes[raySceneIntersection.objectIndex].material.type == Material_Phong) {
                    ComputePhongMaterial_Mesh(color, meshes[raySceneIntersection.objectIndex], raySceneIntersection, ray, ambiant, intensiteAmbiant, diffuse, intensiteDiffuse, speculaire, intensiteSpeculaire, intensiteDS, L, R, V, estDansOmbre, shadow, nbRayonsOmbres, tailleCarre, marge_Mesh);
                }
                // Réflexion
                if(meshes[raySceneIntersection.objectIndex].material.type == Material_Mirror && NRemainingBounces > 0) {
                    ComputeReflectionMaterial_Mesh(color, raySceneIntersection, ray, NRemainingBounces);                   
                }
                // Réfraction
                if(meshes[raySceneIntersection.objectIndex].material.type == Material_Glass && NRemainingBounces > 0) {
                    ComputeRefractionMaterial_Mesh(color, meshes[raySceneIntersection.objectIndex], raySceneIntersection, ray, NRemainingBounces);                                      
                }
                // Blinn-Phong
                if(meshes[raySceneIntersection.objectIndex].material.type == Material_Blinn_Phong) {
                    ComputeBlinnPhongMaterial_Mesh(color, meshes[raySceneIntersection.objectIndex], raySceneIntersection, ray, ambiant, intensiteAmbiant, diffuse, intensiteDiffuse, speculaire, intensiteSpeculaire, intensiteDS, L, V, H, estDansOmbre, shadow, nbRayonsOmbres, tailleCarre, marge_Mesh);
                }
                // Fresnel / Rim
                if(meshes[raySceneIntersection.objectIndex].material.type == Material_FresnelRim) {
                    ComputeFresnelRimMaterial_Mesh(color, meshes[raySceneIntersection.objectIndex], raySceneIntersection, ray, ambiant, intensiteAmbiant, diffuse, intensiteDiffuse, speculaire, intensiteSpeculaire, intensiteDS, L, V, H, Rim, estDansOmbre, shadow, nbRayonsOmbres, tailleCarre, marge_Mesh, rimFactor, fresnelColor, rimIntensity);
                }
                // Toon
                if(meshes[raySceneIntersection.objectIndex].material.type == Material_Toon) {
                    ComputeToonMaterial_Mesh(color, meshes[raySceneIntersection.objectIndex], raySceneIntersection, ray, ambiant, intensiteAmbiant, diffuse, intensiteDiffuse, speculaire, intensiteSpeculaire, intensiteDS, L, V, H, Rim, estDansOmbre, shadow, nbRayonsOmbres, tailleCarre, marge_Mesh, rimFactor, fresnelColor, rimIntensity, nSteps);
                }
                // PBR
                if(meshes[raySceneIntersection.objectIndex].material.type == Material_PBR) {
                    ComputePBRMaterial_Mesh(color, meshes[raySceneIntersection.objectIndex], raySceneIntersection, ray, ambiant, intensiteAmbiant, diffuse, intensiteDiffuse, speculaire, intensiteSpeculaire, intensiteDS, L, V, H, estDansOmbre, shadow, nbRayonsOmbres, tailleCarre, marge_Mesh, roughness, metallic);
                }
            }
        }

        return color;
    }


    Vec3 rayTrace( Ray const & rayStart ) {
        //TODO appeler la fonction recursive
        Vec3 color;
        color = rayTraceRecursive(rayStart, 5);
        return color;
    }


    /*
    void setup_single_sphere() {
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();

        {
            lights.resize( lights.size() + 1 );
            Light & light = lights[lights.size() - 1];
            light.pos = Vec3(-5,5,5);
            light.radius = 2.5f;
            light.powerCorrection = 2.f;
            light.type = LightType_Spherical;
            light.material = Vec3(1,1,1);
            light.isInCamSpace = false;
        }
        {
            spheres.resize( spheres.size() + 1 );
            Sphere & s = spheres[spheres.size() - 1];
            s.m_center = Vec3(1. , 0.0 , 0.);
            s.m_radius = 1.0f;
            s.build_arrays();
            s.material.type = Material_Basic;
            s.material.diffuse_material = Vec3( 1.,0.,0. );
            s.material.specular_material = Vec3( 0.2,0.2,0.2 );
            s.material.shininess = 20;
        }
        {
            spheres.resize( spheres.size() + 1 );
            Sphere & s = spheres[spheres.size() - 1];
            s.m_center = Vec3(-1. , 0.0 , 0.);
            s.m_radius = 1.0f;
            s.build_arrays();
            s.material.type = Material_Basic;
            s.material.diffuse_material = Vec3( 0.,1.,0. );
            s.material.specular_material = Vec3( 0.2,0.2,0.2 );
            s.material.shininess = 20;
        }
    }
    */

    void setup_single_sphere() {
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();

        {
            lights.resize( lights.size() + 1 );
            Light & light = lights[lights.size() - 1];
            light.pos = Vec3(-5,5,5);
            light.radius = 2.5f;
            light.powerCorrection = 2.f;
            light.type = LightType_Spherical;
            light.material = Vec3(1,1,1);
            light.isInCamSpace = false;
        }
        {
            spheres.resize( spheres.size() + 1 );
            Sphere & s = spheres[spheres.size() - 1];
            s.m_center = Vec3(0. , 0. , 0.);
            s.m_radius = 1.f;
            s.build_arrays();
            s.material.type = Material_Basic;
            s.material.diffuse_material = Vec3( 1.,1.,1 );
            s.material.specular_material = Vec3( 0.2,0.2,0.2 );
            s.material.shininess = 20;
            s.material.textype = Texture_File;
        }
    }


    void setup_single_square() {
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();

        {
            lights.resize( lights.size() + 1 );
            Light & light = lights[lights.size() - 1];
            light.pos = Vec3(-5,5,5);
            light.radius = 2.5f;
            light.powerCorrection = 2.f;
            light.type = LightType_Spherical;
            light.material = Vec3(1,1,1);
            light.isInCamSpace = false;
        }

        {
            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.build_arrays();
            s.material.type = Material_Basic;
            s.material.diffuse_material = Vec3( 0.8,0.8,0.8 );
            s.material.specular_material = Vec3( 0.8,0.8,0.8 );
            s.material.shininess = 20;
            s.material.textype = Texture_File;
        }
    }

    void setup_single_triangle() {
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();

        {
            lights.resize( lights.size() + 1 );
            Light & light = lights[lights.size() - 1];
            light.pos = Vec3(-5,5,5);
            light.radius = 2.5f;
            light.powerCorrection = 2.f;
            light.type = LightType_Spherical;
            light.material = Vec3(1,1,1);
            light.isInCamSpace = false;
        }

        {
            meshes.resize( meshes.size() + 1 );
            Mesh & m = meshes[meshes.size() - 1];
            m.loadOFF("./data/triangle.off");
            m.centerAndScaleToUnit();
            m.build_arrays();
            m.material.type = Material_Basic;
            m.material.diffuse_material = Vec3( 1.,1.,1. );
            m.material.specular_material = Vec3( 0.8,0.8,0.8 );
            m.material.shininess = 20;
        }
    }

    void setup_single_mesh() {
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();

        {
            lights.resize( lights.size() + 1 );
            Light & light = lights[lights.size() - 1];
            light.pos = Vec3(-5,5,5);
            light.radius = 2.5f;
            light.powerCorrection = 2.f;
            light.type = LightType_Spherical;
            light.material = Vec3(1,1,1);
            light.isInCamSpace = false;
        }

        {
            meshes.resize( meshes.size() + 1 );
            Mesh & m = meshes[meshes.size() - 1];
            m.loadOFF("./data/cube.off");
            m.centerAndScaleToUnit();
            m.build_arrays();
            m.material.type = Material_Basic;
            m.material.diffuse_material = Vec3( 0.8,0.2,0.3 );
            m.material.specular_material = Vec3( 0.8,0.8,0.8 );
            m.material.shininess = 20;
        }
    }

    void setup_cornell_box(){
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();

        {
            lights.resize( lights.size() + 1 );
            Light & light = lights[lights.size() - 1];
            light.pos = Vec3(0.0, 1.55, 0.75);
            light.radius = 2.5f;
            light.powerCorrection = 2.f;
            light.type = LightType_Quad;
            light.material = Vec3(1,1,1);
            light.isInCamSpace = false;
        }

        { //Back Wall
            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1.75, -1.75, 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 3.5, 3.5);
            s.build_arrays();
            s.material.type = Material_Texture;
            //s.material.type = Material_Basic;
            s.material.diffuse_material = Vec3( 1.,1.,1. );
            s.material.specular_material = Vec3( 1.,1.,1. );
            s.material.shininess = 16;
            s.material.transparency = 1.;
            s.material.index_medium = 1.;
            s.material.textype = Texture_File;
            s.material.nbRepetitions = 10;
        }

        { //Left Wall
            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1.75, -1.75, 0.), Vec3(0., 1.5, 0.), Vec3(0., 0., 1.), 3.5, 3.5);
            s.build_arrays();
            s.material.type = Material_Blinn_Phong;
            //s.material.type = Material_Basic;
            s.material.diffuse_material = Vec3( 1.,0.,0. );
            s.material.specular_material = Vec3( 1.,0.,0. );
            s.material.shininess = 16;
            s.material.transparency = 1.;
            s.material.index_medium = 1.;
        }

        { //Right Wall
            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(1.75, -1.75, 0.), Vec3(0., 0., 1.), Vec3(0., 1.5, 0.), 3.5, 3.5);
            s.build_arrays();
            s.material.type = Material_Blinn_Phong;
            //s.material.type = Material_Basic;
            s.material.diffuse_material = Vec3( 0.0,1.0,0.0 );
            s.material.specular_material = Vec3( 1.,0.,0. );
            s.material.shininess = 16;
            s.material.transparency = 1.;
            s.material.index_medium = 1.;
        }

        { //Ceil Wall
            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1.75, 1.75, 0.), Vec3(1., 0, 0.), Vec3(0., 0, 1.), 3.5, 3.5);
            s.build_arrays();
            s.material.type = Material_Blinn_Phong;
            //s.material.type = Material_Basic;
            s.material.diffuse_material = Vec3( 0.7,0.8,0.5 );
            s.material.specular_material = Vec3( 1.,1.,1. );
            s.material.shininess = 16;
            s.material.transparency = 1.;
            s.material.index_medium = 1.;
        }

        { //Floor Wall
            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1.75, -1.75, 0.), Vec3(0., 0, 1.), Vec3(1., 0, 0.), 3.5, 3.5);
            s.build_arrays();
            s.material.type = Material_Blinn_Phong;
            //s.material.type = Material_Basic;
            s.material.diffuse_material = Vec3( 0.5,0.9,1.0 );
            s.material.specular_material = Vec3( 1.,1.,1. );
            s.material.shininess = 16;
            s.material.transparency = 1.;
            s.material.index_medium = 1.;
        }
        
        /*
        {
            spheres.resize( spheres.size() + 1 );
            Sphere & s = spheres[spheres.size() - 1];
            s.m_center = Vec3(0.85, -1.05, 1.25);
            s.m_radius = 0.65f;
            s.build_arrays();
            //s.material.type = Material_Glass;
            //s.material.type = Material_PBR;
            //s.material.type = Material_Texture;
            s.material.type = Material_Texture;
            //s.material.type = Material_Basic;
            s.material.diffuse_material = Vec3( 1.,0.,0. );
            s.material.specular_material = Vec3( 1.,0.,0. );
            s.material.shininess = 16;
            //s.material.transparency = 1.0;
            //s.material.index_medium = 1.4;
            s.material.transparency = 1.0;
            s.material.index_medium = 1.4;
            s.material.textype = Texture_File;
            s.material.nbRepetitions = 10;
        }
        */
        
        /*
        { //GLASS Sphere
            spheres.resize( spheres.size() + 1 );
            Sphere & s = spheres[spheres.size() - 1];
            s.m_center = Vec3(0.85, -1.05, 1.25);
            s.m_radius = 0.65f;
            s.build_arrays();
            //s.material.type = Material_Glass;
            //s.material.type = Material_PBR;
            //s.material.type = Material_Glass;
            s.material.type = Material_PBR;
            //s.material.type = Material_Phong;
            //s.material.type = Material_Basic;
            s.material.diffuse_material = Vec3( 1.,0.,0. );
            s.material.specular_material = Vec3( 1.,0.,0. );
            s.material.shininess = 16;
            //s.material.transparency = 1.0;
            //s.material.index_medium = 1.4;
            s.material.transparency = 1.0;
            s.material.index_medium = 2.0;
        }
        
        { //MIRRORED Sphere
            spheres.resize( spheres.size() + 1 );
            Sphere & s = spheres[spheres.size() - 1];
            s.m_center = Vec3(-0.85, -1.05, 0.8);
            s.m_radius = 0.65f;
            s.build_arrays();
            //s.material.type = Material_PBR;
            //s.material.type = Material_Glass;
            s.material.type = Material_PBR;
            //s.material.type = Material_Glass;
            //s.material.type = Material_Basic;
            s.material.diffuse_material = Vec3( 1.,1.,1. );
            s.material.specular_material = Vec3(  1.,1.,1. );
            s.material.shininess = 16;
            //s.material.transparency = 0.;
            //s.material.index_medium = 0.;
            s.material.transparency = 1.;
            s.material.index_medium = 3.;
        }
        */

        /*
        {
            meshes.resize( meshes.size() + 1 );
            Mesh & m = meshes[meshes.size() - 1];
            m.loadOFF("data/suzanne.off");
            m.centerAndScaleToUnit();
            m.scale(Vec3(0.5, 0.5, 0.5));
            //m.translate(Vec3(0., -1., 0.75));
            m.translate(Vec3(0., -0.85, 1.05));
            //m.rotate_x(60);
            m.build_arrays();
            m.material.type = Material_Blinn_Phong;
            m.material.diffuse_material = Vec3( 0.3,0.4,0.8 );
            m.material.specular_material = Vec3( 0.8,0.8,0.8 );
            m.material.shininess = 16;
            m.material.transparency = 1.;
            m.material.index_medium = 1.;

            // AABB
            AABB bbmesh = optimize(m);
            bbox.push_back(bbmesh);
        }  
        */    
    }
};

#endif
