#ifndef SQUARE_H
#define SQUARE_H
#include "Vec3.h"
#include <vector>
#include "Mesh.h"
#include <cmath>

struct RaySquareIntersection{
    bool intersectionExists;
    float t;
    float u,v;
    Vec3 intersection;
    Vec3 normal;
};


class Square : public Mesh {
public:
    Vec3 m_normal;
    Vec3 m_bottom_left;
    Vec3 m_right_vector;
    Vec3 m_up_vector;

    Square() : Mesh() {}
    Square(Vec3 const & bottomLeft , Vec3 const & rightVector , Vec3 const & upVector , float width=1. , float height=1. ,
           float uMin = 0.f , float uMax = 1.f , float vMin = 0.f , float vMax = 1.f) : Mesh() {
        setQuad(bottomLeft, rightVector, upVector, width, height, uMin, uMax, vMin, vMax);
    }

    void setQuad( Vec3 const & bottomLeft , Vec3 const & rightVector , Vec3 const & upVector , float width=1. , float height=1. ,
                  float uMin = 0.f , float uMax = 1.f , float vMin = 0.f , float vMax = 1.f) {
        m_right_vector = rightVector;
        m_up_vector = upVector;
        m_normal = Vec3::cross(rightVector , upVector);
        m_bottom_left = bottomLeft;

        m_normal.normalize();
        m_right_vector.normalize();
        m_up_vector.normalize();

        m_right_vector = m_right_vector*width;
        m_up_vector = m_up_vector*height;

        vertices.clear();
        vertices.resize(4);
        vertices[0].position = bottomLeft;                                      vertices[0].u = uMin; vertices[0].v = vMin;
        vertices[1].position = bottomLeft + m_right_vector;                     vertices[1].u = uMax; vertices[1].v = vMin;
        vertices[2].position = bottomLeft + m_right_vector + m_up_vector;       vertices[2].u = uMax; vertices[2].v = vMax;
        vertices[3].position = bottomLeft + m_up_vector;                        vertices[3].u = uMin; vertices[3].v = vMax;
        vertices[0].normal = vertices[1].normal = vertices[2].normal = vertices[3].normal = m_normal;
        triangles.clear();
        triangles.resize(2);
        triangles[0][0] = 0;
        triangles[0][1] = 1;
        triangles[0][2] = 2;
        triangles[1][0] = 0;
        triangles[1][1] = 2;
        triangles[1][2] = 3;
    }

    RaySquareIntersection intersect(const Ray &ray) const {
        RaySquareIntersection intersection;

        //TODO calculer l'intersection rayon quad
        Vec3 inter = Vec3(0., 0., 0.);
        intersection.intersectionExists = false;

        // Coordonnées en X, Y
        float u, v;

        // Pré-calcul de t
        float D = Vec3::dot(m_bottom_left, m_normal);
        float ON = Vec3::dot(ray.origin(), m_normal);
        float DN = Vec3::dot(ray.direction(), m_normal);

        float num = D - ON;
        float denum = DN;

        double seuilOmbre = 0.0001;

        // Calcul de t
        float t = (num) / (denum);

        // J'ignore toutes les valeurs en dessous de seuilOmbre (pour l'ombrage)
        if(t < seuilOmbre) {
            intersection.intersectionExists = false;
            return intersection;
        }
        
        // Vérification que l'intersection est devant
        if(t > 0.0f) {
            // Calcul de l'intersection
            inter = ray.origin() + t * ray.direction();

            // Délimitation des "bords" du plan
            u = Vec3::dot(inter - m_bottom_left, m_right_vector) / Vec3::dot(m_right_vector, m_right_vector);   // Coordonnée X
            v = Vec3::dot(inter - m_bottom_left, m_up_vector) / Vec3::dot(m_up_vector, m_up_vector);            // Coordonnée Y

            if (u >= 0. && u <= 1. && v >= 0. && v <= 1.) {     // Vérification des limites du plan
                intersection.intersection = inter;
                intersection.intersectionExists = true;
                intersection.normal = m_normal;
                intersection.t = t;
                intersection.u = u;
                intersection.v = v;
                return intersection;
            } 
        } 

        return intersection;
    }
};
#endif // SQUARE_H
