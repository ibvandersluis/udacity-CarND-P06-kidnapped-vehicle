/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 * 
 * Modified on: Sep 30, 2022
 * Maintainer: Isaac Vander Sluis
 */

#include "particle_filter/particle_filter.h"

#include <math.h>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "particle_filter/helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[])
{
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */

  std::default_random_engine gen;

  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);

  num_particles_ = 1000;  // TODO: Set the number of particles

  for (unsigned int i = 0; i < num_particles_; ++i) {
    Particle p;

    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1.0;

    particles_.push_back(p);
  }
  initialized_ = true;
}

void ParticleFilter::prediction(double d_t, double std_pos[], double velocity, double yaw_rate)
{
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  std::default_random_engine gen;

  for (auto & p : particles_) {
    auto theta_f = double{};
    auto x_f = double{};
    auto y_f = double{};

    if (abs(yaw_rate) < 0.0001) {  // If yaw rate is effectively 0
      theta_f = p.theta;
      x_f = p.x + velocity * d_t * cos(p.theta);
      y_f = p.y + velocity * d_t * sin(p.theta);
    } else {
      theta_f = p.theta + yaw_rate * d_t;
      x_f = p.x + ((velocity / yaw_rate) * (sin(theta_f) - sin(p.theta)));
      y_f = p.y + ((velocity / yaw_rate) * (cos(p.theta) - cos(theta_f)));
    }

    std::normal_distribution<double> dist_x(x_f, std_pos[0]);
    std::normal_distribution<double> dist_y(y_f, std_pos[1]);
    std::normal_distribution<double> dist_theta(theta_f, std_pos[2]);

    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(
  vector<LandmarkObs> predicted, vector<LandmarkObs> & observations)
{
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

  for (size_t j = 0; j < observations.size(); ++j) {
    auto x1 = observations.at(j).x;
    auto y1 = observations.at(j).y;
    int closest = 0;
    double min_distance;
    for (size_t k = 0; k < predicted.size(); ++k) {
      auto x2 = predicted.at(k).x;
      auto y2 = predicted.at(k).y;
      auto distance = dist(x1, y1, x2, y2);

      if (k == 0) {
        min_distance = distance;
        closest = predicted.at(k).id;
      } else if (distance < min_distance) {
        min_distance = distance;
        closest = predicted.at(k).id;
      }
    }
    observations.at(j).id = closest;
  }
}

void ParticleFilter::updateWeights(
  double /* sensor_range */, double std_landmark[], const vector<LandmarkObs> & observations,
  const Map & map_landmarks)
{
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  weights_.clear();

  for (auto & p : particles_) {
    auto tf_observations = vector<LandmarkObs>{};

    for (auto & obs : observations) {
      auto tf_x = p.x + (cos(p.theta) * obs.x) - (sin(p.theta) * obs.y);
      auto tf_y = p.y + (sin(p.theta) * obs.x) + (cos(p.theta) * obs.y);

      auto tf_obs = LandmarkObs{obs.id, tf_x, tf_y};

      tf_observations.emplace_back(tf_obs);
    }
    // Landmarks must be a vector of osbervations for data association
    auto predicted = std::vector<LandmarkObs>{};

    for (auto & lm_map : map_landmarks.landmark_list) {
      auto lm = LandmarkObs{lm_map.id_i, lm_map.x_f, lm_map.y_f};
      predicted.emplace_back(lm);
    }
    // TODO: Step 2 - Associate transformed observations with nearest landmark
    dataAssociation(predicted, tf_observations);

    auto weight = 1.0;

    for (auto & obs : tf_observations) {
      // TODO: Step 3a - Calculate probabilities
      auto lm = getLandmarkById(obs.id, map_landmarks);
      auto probability = mv_pdf(obs.x, obs.y, lm.x_f, lm.y_f, std_landmark[0], std_landmark[1]);

      // TODO: Step 3b - Combine probabilities
      weight *= probability;
    }
    p.weight = weight;
    weights_.emplace_back(weight);
  }
}

void ParticleFilter::resample()
{
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  std::random_device rd;
  std::mt19937 gen(rd());
  std::discrete_distribution<> d(weights_.begin(), weights_.end());

  auto old_particles = particles_;
  particles_.clear();

  for (size_t i = 0; i < old_particles.size(); ++i) {
    particles_.emplace_back(old_particles.at(d(gen)));
  }
}

void ParticleFilter::SetAssociations(
  Particle & particle, const vector<int> & associations, const vector<double> & sense_x,
  const vector<double> & sense_y)
{
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord)
{
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}

Map::single_landmark_s ParticleFilter::getLandmarkById(unsigned int id, const Map & map)
{
  for (size_t i = 0; i < map.landmark_list.size(); ++i) {
    if (map.landmark_list.at(i).id_i == id) return map.landmark_list.at(i);
  }

  throw std::runtime_error("No matching landmark found");
}
