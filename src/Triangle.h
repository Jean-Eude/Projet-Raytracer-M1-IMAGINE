#ifndef TRIANGLE_H
#define TRIANGLE_H
#include "Vec3.h"
#include "Ray.h"
#include "Plane.h"

struct RayTriangleIntersection{
    bool intersectionExists;
    float t;
    float w0,w1,w2;
    unsigned int tIndex;
    Vec3 intersection;
    Vec3 normal;
};

class Triangle {
private:
    Vec3 m_c[3] , m_normal;
    float area;
public:
    Triangle() {}
    Triangle( Vec3 const & c0 , Vec3 const & c1 , Vec3 const & c2 ) {
        m_c[0] = c0;
        m_c[1] = c1;
        m_c[2] = c2;
        updateAreaAndNormal();
    }
    void updateAreaAndNormal() {
        Vec3 nNotNormalized = Vec3::cross( m_c[1] - m_c[0] , m_c[2] - m_c[0] );
        float norm = nNotNormalized.length();
        m_normal = nNotNormalized / norm;
        area = norm / 2.f;
    }
    void setC0( Vec3 const & c0 ) { m_c[0] = c0; } // remember to update the area and normal afterwards!
    void setC1( Vec3 const & c1 ) { m_c[1] = c1; } // remember to update the area and normal afterwards!
    void setC2( Vec3 const & c2 ) { m_c[2] = c2; } // remember to update the area and normal afterwards!
    Vec3 const & normal() const { return m_normal; }
    Vec3 projectOnSupportPlane( Vec3 const & p ) const {
        Vec3 result;
        //TODO completer
        return result;
    }
    float squareDistanceToSupportPlane( Vec3 const & p ) const {
        float result;
        //TODO completer
        return result;
    }

    float distanceToSupportPlane( Vec3 const & p ) const { 
        return sqrt( squareDistanceToSupportPlane(p) ); 
    }

    bool isParallelTo( Line const & L ) const {
        bool result = false;
        double isParallel = Vec3::dot(L.direction(), this->m_normal);   // Vérification si parallèle ou non
        result = (isParallel >= 0.0);                                   // renvoie true si >= 0. sinon false
        return result;
    }

    float getIntersectionPointWithSupportPlane( Line const & L) const {
        // you should check first that the line is not parallel to the plane!
        float result = 0.0f;
        
        if(!isParallelTo(L)) {
            double num, denum;
            num = Vec3::dot(this->m_c[0] - L.origin(), this->m_normal);
            denum = Vec3::dot(L.direction(), this->m_normal);

            float t = num / denum;    // Formule pour calculer : t = ((a-p).n / d.n) 

            // J'attribue la valeur de t à result ssi t > 0 et donc que si l'intersection est devant.
            if(t >= 0.00000001) {
                result = t;
            }
        }

        return result;
    }

    void computeBarycentricCoordinates( Vec3 const & p , float & u0 , float & u1 , float & u2) const {
        //TODO Complete

        // Calcul des produits vectoriels pour calculer l'aire de chaque "sous-triangle"
        float aire0 = Vec3::cross(p - this->m_c[0], this->m_c[1] - this->m_c[0]).length()/2.f;
        float aire1 = Vec3::cross(p - this->m_c[1], this->m_c[2] - this->m_c[1]).length()/2.f;
        float aire2 = Vec3::cross(p - this->m_c[2], this->m_c[0] - this->m_c[2]).length()/2.f;

        // Calcul des coordonnées barycentriques
        u0 = aire0/this->area; 
        u1 = aire1/this->area; 
        u2 = aire2/this->area;
    }

    RayTriangleIntersection getIntersection( Ray const & ray ) const {
        RayTriangleIntersection result;

        float t;
        double seuilOmbre = 0.001;

        // On récupère la valeur retournée par la fonction donc t
        t = getIntersectionPointWithSupportPlane(ray);

        // Calcul de l'intersection
        Vec3 inter = ray.origin() + t * ray.direction();

        // Vérification si le rayon est parallèle au triangle
        if(isParallelTo(ray)) {
            result.intersectionExists = false;
            return result;
        }

        // J'ignore toutes les valeurs en dessous de seuilOmbre (pour l'ombrage)
        if(t < seuilOmbre) {
            result.intersectionExists = false;
            return result;
        }
        
        // Calcul des coordonnées barycentriques
        float u0, u1, u2;
        computeBarycentricCoordinates(inter, u0, u1 , u2);
        
        // Test d'appartenance si les coordonnées sont >= 0 et a+b+y <= 1 et >= 0
        if(u0 >= 0. && u1 >= 0. && u2 >= 0. && (u0 + u1 + u2 >= 1 - 0.0001) && (u0 + u1 + u2 <= 1 + 0.0001)) {
            result.intersectionExists = true;
            result.t = t;
            result.w0 = u0;
            result.w1 = u1;
            result.w2 = u2;
            result.intersection = inter;  
            result.normal = this->m_normal;
            result.normal.normalize();
        } else {
            result.intersectionExists = false;
            return result;
        }

        return result;
    }
};
#endif
