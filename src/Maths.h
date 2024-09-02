double clamp(double x, double min, double max)  { 
    
    if(x < min) {
        x = min;
    }
    else if(x > max) {
        x = max;
    }

    return x;
}

double step(double a, double x)   {
    
    if(a < x) {
        return 1.0;
    }
    
    return 0.0;
}

double smoothstep(double e0, double e1, double x)   {
    
    x = clamp((x - e0) / (e1 - e0), 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

double fract(double x) {
    return x-(long)x;
}
