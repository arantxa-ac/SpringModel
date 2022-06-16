#include "models.h"

#include <memory>

namespace simulation {

//
// Mass on a Spring
//
MassOnASpringModel::MassOnASpringModel() { reset(); }

void MassOnASpringModel::reset() {
    particles.clear();
    springs.clear();
    // create fixed particle
    std::shared_ptr<Particle> p0(new Particle({ 0.f, 5.f, 0.f }, vec3f{ 0.f }, 0.f, vec3f{ 0.f }));
    particles.push_back(p0);
    // create moving particle
    std::shared_ptr<Particle> p1(new Particle({ 0.f, -5.f, 0.f }, vec3f{ 0.f }, mass, vec3f{ 0.f }));
    particles.push_back(p1);
    // create spring
    springs.push_back(Spring(p0, p1, ks, kd)); 
    //compress spring
    p1->x = vec3f{ 0.f, 2.5f, 0.f };
}

void MassOnASpringModel::step(float dt) {
    // calculate spring forces
    std::shared_ptr<Particle> pi = springs[0].pi;
    std::shared_ptr<Particle> pj = springs[0].pj;
    vec3f Fs_ij = -1.f * springs[0].ks * (distance(pj->x, pi->x) - springs[0].l) * (pj->x - pi->x) / distance(pj->x, pi->x);
    vec3f Fd_ij;
    if (Fs_ij == vec3f{ 0.f }) {
        vec3f dVector = pj->x - pi->x;
        Fd_ij = -1.f * springs[0].kd * dot((pj->v - pi->v), normalize(dVector)) / dot(normalize(dVector), normalize(dVector)) * normalize(dVector);
    }
    else
        Fd_ij = -1.f * springs[0].kd * (dot((pj->v - pi->v), normalize(Fs_ij)) / dot(normalize(Fs_ij), normalize(Fs_ij))) * normalize(Fs_ij);
    particles[1]->F += Fs_ij + Fd_ij;
    // semi-implicit Euler integration for moving particle
    particles[1]->v += (particles[1]->F / particles[1]->m) * dt;
    particles[1]->x += particles[1]->v * dt;
    particles[1]->F = vec3f{0.f};
}

//
// Chain Pendulum
//
ChainPendulumModel::ChainPendulumModel() { reset(); }

void ChainPendulumModel::reset() {
    particles.clear();
    springs.clear();
    //create fixed particle
    std::shared_ptr<Particle> pf(new Particle({ 0.f, 10.f, 0.f }, vec3f{ 0.f }, 0.f, vec3f{ 0.f }));
    particles.push_back(pf);
    // create particles and springs
    for (int i = 0; i < 10; i++) {
        std::shared_ptr<Particle> p(new Particle({ 2 * (i + 1), 10.f, 0.f }, vec3f{ 0.f }, mass, vec3f{ 0.f }));
        particles.push_back(p);
        springs.push_back(Spring(pf, p, ks, kd));
        pf = p;
    }
}

void ChainPendulumModel::step(float dt) {
    // calculate spring forces
    for (auto& spring : springs) {
        // for particle i
        vec3f Fs_ij = -1.f * spring.ks * (distance(spring.pi->x, spring.pj->x) - spring.l) * normalize(spring.pi->x - spring.pj->x);
        vec3f Fd_ij;
        if (Fs_ij == vec3f{ 0.f }) {
            vec3f dVector = spring.pi->x - spring.pj->x;
            Fd_ij = -1.f * springs[0].kd * dot((spring.pi->v - spring.pj->v), normalize(dVector)) / dot(normalize(dVector), normalize(dVector)) * normalize(dVector);
        }
        else
            Fd_ij = -1.f * springs[0].kd * (dot((spring.pi->v - spring.pj->v), normalize(Fs_ij)) / dot(normalize(Fs_ij), normalize(Fs_ij))) * normalize(Fs_ij);
        spring.pi->F += Fs_ij + Fd_ij;
        // for particle j
        spring.pj->F -= (Fs_ij + Fd_ij);
    }
    for (auto& p : particles) {
        // add gravity
        p->F += p->m * gravity;
        // semi-implicit Euler integration for moving particles
        if (p->m > 0.f) {
            p->v += (p->F / p->m) * dt;
            p->x += p->v * dt;
            p->F = vec3f{0.f};
        }
    }
}

//
// Cloth Model
//
ClothModel::ClothModel() { reset(); }

void ClothModel::reset() {
    particles.clear();
    springs.clear();
    // create particles
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            float m = mass;
            if ((i == 0 && j == 0) || (i == 0 && j == 9)) m = 0.f; //set fixed particles
            std::shared_ptr<Particle> p(new Particle({ 2 * (i - 5.f), 0.f, 2 * (j - 5.f) }, vec3f{ 0.f }, m, vec3f{ 0.f }));
            particles.push_back(p);
        }
    }
    // create springs
    for (int i = 0; i < particles.size(); i++) {
        for (int j = 0; j < i; j++) {
            vec3f d = abs(particles[i]->x - particles[j]->x);
            if ((d.x <= 2.f) && (d.z <= 2.f)) {
                springs.push_back(Spring(particles[i], particles[j], ks, kd));
            }
                
        }
    }
}

void ClothModel::step(float dt) {

    // calculate spring forces
    for (auto& spring : springs) {
        // for particle i
        vec3f Fs_ij = -1.f * spring.ks * (distance(spring.pi->x, spring.pj->x) - spring.l) * normalize(spring.pi->x - spring.pj->x);
        vec3f Fd_ij;
        if (Fs_ij == vec3f{ 0.f }) {
            vec3f dVector = spring.pi->x - spring.pj->x;
            Fd_ij = -1.f * springs[0].kd * dot((spring.pi->v - spring.pj->v), normalize(dVector)) / dot(normalize(dVector), normalize(dVector)) * normalize(dVector);
        }
        else
            Fd_ij = -1.f * springs[0].kd * (dot((spring.pi->v - spring.pj->v), normalize(Fs_ij)) / dot(normalize(Fs_ij), normalize(Fs_ij))) * normalize(Fs_ij);
        spring.pi->F += Fs_ij + Fd_ij;
        // for particle j
        spring.pj->F -= (Fs_ij + Fd_ij);
    }
    for (auto& p : particles) {
        // add gravity
        p->F += p->m * gravity;
        // semi-implicit Euler integration for moving particles
        if (p->m > 0.f) {
            p->v += (p->F / p->m) * dt;
            p->x += p->v * dt;
            p->F = vec3f{ 0.f };
        }
    }
}

//
// Jelly Cube Model
//
CubeModel::CubeModel() { reset(); }

void CubeModel::reset() {
    particles.clear();
    springs.clear();
    // create particles
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            for (int k = 0; k < 6; k++) {
                for (int h = 0; h < 6; h++) {
                    std::shared_ptr<Particle> p(new Particle({ 2 * (i), 2 * (h), 2 * (j - 5.f) }, vec3f{ 0.f }, mass, vec3f{ 0.f }));
                    particles.push_back(p);
                }
            }
        }
    }
    // create springs
    for (int i = 0; i < particles.size(); i++) {
        for (int j = 0; j < i; j++) {
            vec3f d = abs(particles[i]->x - particles[j]->x);
            if ((d.x <= 2.f) && (d.y <= 2.f) && (d.z <= 2.f)) {
                springs.push_back(Spring(particles[i], particles[j], ks, kd));
            }

        }
    }
}

void CubeModel::step(float dt) {
    // calculate spring forces
    for (auto& spring : springs) {
        // for particle i
        vec3f Fs_ij = -1.f * spring.ks * (distance(spring.pi->x, spring.pj->x) - spring.l) * normalize(spring.pi->x - spring.pj->x);
        vec3f Fd_ij;
        if (Fs_ij == vec3f{ 0.f }) {
            vec3f dVector = spring.pi->x - spring.pj->x;
            Fd_ij = -1.f * springs[0].kd * dot((spring.pi->v - spring.pj->v), normalize(dVector)) / dot(normalize(dVector), normalize(dVector)) * normalize(dVector);
        }
        else
            Fd_ij = -1.f * springs[0].kd * (dot((spring.pi->v - spring.pj->v), normalize(Fs_ij)) / dot(normalize(Fs_ij), normalize(Fs_ij))) * normalize(Fs_ij);
        spring.pi->F += Fs_ij + Fd_ij;
        // for particle j
        spring.pj->F -= (Fs_ij + Fd_ij);
    }
    for (auto& p : particles) {
        // add gravity
        p->F += p->m * gravity;
        // semi-implicit Euler integration for moving particles
        if (p->m > 0.f) {
            p->v += (p->F / p->m) * dt;
            p->x += p->v * dt;
            p->F = vec3f{ 0.f };
        }
    }
}

} // namespace simulation
