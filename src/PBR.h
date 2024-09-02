// Distribution normale
double DistributionGGX(Vec3 &N, Vec3 &H, float a)   {
    float a2     = a*a;
    float NdotH  = std::max(Vec3::dot(N, H), 0.0f);
    float NdotH2 = NdotH*NdotH;
        
    float nom    = a2;
    float denom  = (NdotH2 * (a2 - 1.0) + 1.0);
    denom        = M_PI * denom * denom;
        
    return nom / denom;
}

// Fonction géométrique
float GeometrySchlickGGX(float NdotV, float k)  {
    float nom   = NdotV;
    float denom = NdotV * (1.0 - k) + k;
        
    return nom / denom;
}
    
float GeometrySmith(Vec3 &N, Vec3 &V, Vec3 &L, float k) {
    float NdotV = std::max(Vec3::dot(N, V), 0.0f);
    float NdotL = std::max(Vec3::dot(N, L), 0.0f);
    float ggx1 = GeometrySchlickGGX(NdotV, k);
    float ggx2 = GeometrySchlickGGX(NdotL, k);
        
    return ggx1 * ggx2;
}

// Approximation de Schlick
double F(float n1, float n2, float exponent, Vec3 &V, Vec3 &H) {
    double F0 = (n1 - n2) * (n1 - n2) / (n1 + n2) * (n1 + n2);
    return F0 + (1. - F0) * std::max(std::pow(1. - Vec3::dot(V, H), exponent), 0.);
}