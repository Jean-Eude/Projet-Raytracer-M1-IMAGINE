#include <vector>
#include <string>
#include "Mesh.h"
#include "Sphere.h"
#include "Square.h"
#include "Triangle.h"
#include <cmath>



class AABB {
    private:
        Vec3 AABB_min, AABB_max;

    public:

        // Constructeurs
        AABB(Vec3 &min, Vec3 &max) : AABB_min(min), AABB_max(max) {}
        AABB() : AABB_min(Vec3(0., 0., 0.)), AABB_max(Vec3(0., 0., 0.)) {}

        // Intersection de la boîte
        bool intersect(const Ray &ray) const {
            float t_min = std::numeric_limits<float>::lowest();
            float t_max = std::numeric_limits<float>::max();

            for (int i = 0; i < 3; i++) {
                float t1 = (this->AABB_min[i] - ray.origin()[i]) / ray.direction()[i];
                float t2 = (this->AABB_max[i] - ray.origin()[i]) / ray.direction()[i];

                if (t1 > t2) {
                    std::swap(t1, t2);
                }

                t_min = std::max(t_min, t1);
                t_max = std::min(t_max, t2);

                if (t_max < t_min) {
                    return false;
                }
            }

            return true;
        }

        // Fonction pour connaitre les limites de la boîte
        std::pair<Vec3, Vec3> getBords(Mesh &mesh) {
            Vec3 min_Bounds = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
            Vec3 max_Bounds = {std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()};

            for (unsigned int i : mesh.triangles_array) {
                MeshVertex &vertex = mesh.vertices[i];
                for (int j = 0; j < 3; j++) {
                    min_Bounds[j] = std::min(min_Bounds[j], vertex.position[j]);
                    max_Bounds[j] = std::max(max_Bounds[j], vertex.position[j]);
                }
            }

            return std::make_pair(min_Bounds, max_Bounds);
        }

        void expansion(AABB &other) {
            for (int i = 0; i < 3; i++) {
                this->AABB_min[i] = std::min(this->AABB_min[i], other.AABB_min[i]);
                this->AABB_max[i] = std::max(this->AABB_max[i], other.AABB_max[i]);
            }
        }

        Vec3 getMin() {
            return this->AABB_min;
        }

        Vec3 getMax() {
            return this->AABB_max;
        }

        void setMin(Vec3 min) {
            this->AABB_min = min;
        }

        void setMax(Vec3 max) {
            this->AABB_max = max;
        }
};

AABB optimize(Mesh &m) {
    AABB boundingBoxMesh;
    std::pair<Vec3, Vec3> boundsMesh = boundingBoxMesh.getBords(m);
    Vec3 min = boundsMesh.first;
    Vec3 max = boundsMesh.second;
    AABB boundingBoxBound(min, max);
    boundingBoxMesh.expansion(boundingBoxBound);
    return boundingBoxMesh;            
}