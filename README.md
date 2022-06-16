Spring Model Assignment
-----------------------
Assignment for Fundamentals of Computer Animation course. The purpose of this assignment was to simulate three different spring models using OpenGL and GIVR APIs.
The models developed are described below.

-Mass on a spring: This first model consist of a fixed particle attached to another particle through a spring. Arbitrary values for spring constant and mass of the particle as used to produce the effect seen in the video above.

-Chain pendulum: For this model a fixed particle is attached to a series of 10 particles connected by springs to create a bouncy chain pendulum. Again, arbitrary values are used for spring constant and masses.

-Hanging cloth: The last model is a spring mesh where a plane made up of particles is connected with springs to create the effect of a cloth. This cloth is fixed on two corners and affected by gravity on all other particles.

Documentation
-------------

https://lakin.ca/givr/

Compilation
-----------

This program requires the glm and glfw libraries

## How to Install Dependencies (Ubuntu)

    sudo apt install cmake build-essential

## How to Build

    cmake -H. -Bbuild -DCMAKE_BUILD_TYPE=Release
    cmake --build build

## How to Run

    build/simple
    
    activate panel using key P and use interactive buttons
