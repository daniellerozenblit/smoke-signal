#ifndef SMOKE_H
#define SMOKE_H


class Smoke
{
public:
    Smoke();

    void emitSmoke();
    void advectVelocity();
    void calculateForces();
    void projectPressure();
    void advectTemp();
    void advectDensity();
};

#endif // SMOKE_H
