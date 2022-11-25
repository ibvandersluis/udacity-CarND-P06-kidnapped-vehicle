# Particle Filter in C++

The sixth project in the Udacity Self-Driving Car Engineer Nanodegree: using an partic filter in C++ to estimate localize a moving vehicle against observed landmarks.

## Description

The particle filter algorithm works using the following algorithm:

![Particle Filter Algorithm Flowchart](https://user-images.githubusercontent.com/14826664/193294008-6de88409-48c1-40af-ae91-304e7890a90c.png)

The steps that are critical to executing this algorithm correctly are, in order of execution,

1. Initialisation
2. Prediction
3. Data association
4. Updating weights
5. Resampling with replacement

I'll go through them now in slightly more detail.

### 1. Initialisation

Without any prior information, the particle filter would have to initialise all particles by distributing them randomly across the state space.
However, with this implementation, we have a starting estimate.
This algorithm initialises all particles to the same location, with Gaussian noise added.

### 2. Prediction

We have a vehicle of a known structure, which gives it deterministic motion based on the velocity and the steering angle.
In this case, our motion model is the bicycle model.

![Bicycle Model Diagram](https://user-images.githubusercontent.com/14826664/194922426-686ba820-2d36-4ae4-811d-9bb983e85dc0.png)

This means that, with some small random deviations, we can closely predict where the vehicle will be if we know:

1. How much time has passed
2. The velocity of the vehicle
3. The yaw rate (steering angle) of the vehicle

### 3. Data association

Data association is preformed using the nearest neighbour algorithm, where we make a simple association between the vehicles's predicted landmarks and its obeserved landmarks.
To do this, we transform observed landmarks to the global frame, then for each observation, iterate through all predicted landmarks.
Each observed landmark then identifies the closest predicted landmark as the one it has seen.
These associations may be wildly incorrect, but if that is the case, the particle will have a low weight and is likely to die out.

### 4. Updating weights

With data associations for each observation for each particle, we can now evaluate the likelihood that a given particle is close to the vehicle's true position.
This is computed by evaluating a multi-variate probability density function (PDF) using the pose of the observed landmark, the predicted landmark, and the standard deviations in the x and y directions.
The particle's weight is multiplied by the indivitual multivariate PDF for each observation.
In this way, particles are close to the vehicle's true position will receive higher weights than those that are far off.

### 5. Resampling with replacement

Finally, a new set of particles are resampled from the old set.
The probability of a particle being resampled into the new set is proportionate to that particle's weight compared to the sum of all the weights.
C++ includes a function that conveniently does exactly this, returning the index of an item in a list proportionate to it's value as a percentage of the whole.
The function is called [discrete_distibution](https://en.cppreference.com/w/cpp/numeric/random/discrete_distribution).

After resampling, the algorithm repeats indefinitely (skipping the initialisation step).

## Requirements

- cmake >= 3.10
- make >= 4.1
- gcc/g++ >= 5.4
- uWebSockets

To install uWebSockets, follow these instructions:
```
git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
mkdir build
cd build
cmake ..
make 
sudo make install
cd ../..
sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
sudo rm -r uWebSockets
```

## Build instructions

1. Clone this repo
```
git clone https://github.com/ibvandersluis/udacity-CarND-P06-kidnapped-vehicle.git
cd udacity-CarND-P06-kidnapped-vehicle
```
2. Make build directory
```
mkdir build && cd build
```
3. Build with cmake
```
cmake ..
make
```

## Run

To run this project as intended, you will need the [Term 2 Udacity Simulator](https://github.com/udacity/self-driving-car-sim/releases/).

1. Run the particle filter
```
./particle_filter
```
You should get back:
```
Listening to port 4567
```
2. Run the simulator and select 'Kidnapped Vehicle'

When you do this, the particle filter executable should report:
```
Connected!!!
```
3. Click start. You should see green lines appear between the vehicle and the landmarks. The ground truth of the vehicle is a blue car, and the result of the particle filter is a blue arrow. The arrow should be following the car very closely.

![Kidnapped Robot Simulation](https://user-images.githubusercontent.com/14826664/194925819-6f3a5839-f192-40b7-8f86-706f131c7d4d.png)

> NOTE: If you want to run the particle filter again, click the RESET button but restart the particle filter executable before clicking START again!
