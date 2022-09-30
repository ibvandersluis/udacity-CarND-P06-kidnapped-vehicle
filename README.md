# Particle Filter in C++

The sixth project in the Udacity Self-Driving Car Engineer Nanodegree: using an partic filter in C++ to estimate localize a moving vehicle against observed landmarks.

## Description

The particle filter algorithm works like this:

![Particle Filter Algorithm Flowchart](https://user-images.githubusercontent.com/14826664/193294008-6de88409-48c1-40af-ae91-304e7890a90c.png)

## Requirements

- cmake >= 3.10
- make >= 4.1
- gcc/g++ >= 5.4
- uWebSockets
- [Eigen](https://eigen.tuxfamily.org/index.php)

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
./executable
```
You should get back:
```
Listening to port 4567
```
2. Run the simulator and select '[name]'

When you do this, the particle filter executable should report:
```
Connected!!!
```
3. Select either Dataset 1 or Dataset 2 and click start. You should see laser measurements appear in red and radar measurements appear in blue. The output of the EKF will appear as green triangle markers and should follow the car smoothly. [replace with particle filter instructions]

![Particle Filter Simulator](https://user-images.githubusercontent.com/14826664/183123351-5cafaee7-8ee1-4e20-be41-b2cf7b462bf7.png)

> NOTE: If you want to run the particle filter again, click the RESET button but restart the particle filter executable before clicking START again!
