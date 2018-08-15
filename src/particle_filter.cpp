/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include "Eigen/Dense"

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	int num_particles = 1000;
	double std_x = std[0];
	double std_y = std[1];
	double std_theta = std[2];

	default_random_engine gen;
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	pf.num_particles = num_particles;
	for (int i = 0; i<pf.num_particles, i++){
		//append to the particles vector
		Particle particle;
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
		particle.weight = 1;
		pf.particles.push_back(particle)
	}

	pf.is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	for(int i=0; i<pf.num_particles; i++)
	{
		double std_x = std_pos[0];
		double std_y = std_pos[1];
		double std_theta = std_pos[2];

		default_random_engine gen;
		normal_distribution<double> dist_x(x, std_x);
		normal_distribution<double> dist_y(y, std_y);
		normal_distribution<double> dist_theta(theta, std_theta);

		pf.particles[i].x = pf.particles[i].x  + dist_x(gen) +(velocity/yaw_rate)*(sin(pf.particles[i].theta+yaw_rate*delta_t)-sin(pf.particles[i].theta));
		pf.particles[i].y = pf.particles[i].y  + dist_y(gen) + (velocity/yaw_rate)*(cos(pf.particles[i].theta)-cos(pf.particles[i].theta+yaw_rate*delta_t));
		pf.particles[i].theta = pf.particles[i].theta+ dist_theta(gen) + yaw_rate*delta_t;
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	
	//use euclidian distance to find the nearest neigbor
	double euclidian_dist;
	double prev_euclidian_dist=0;
	double smallest_dist;
	double x1;
	double x2;
	double y1;
	double y2;
	for (int i=0; i<observations.size(); i++){
		x1=observations[i].x;
		//x2=(data from map);
		y1=observations[i].x;
		//y2=(data from map);

		euclidian_dist=(sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1)));
		if (euclidian_dist <  prev_euclidian_dist){
			smallest_dist= euclidian_dist;
		}
		prev_euclidian_dist=euclidian_dist;
	}
	//select nearest neighbor

}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	//transform the observation into map space

	double theta;
	double x_t; 
	double y_t;
	double cummulative_prod;
	double gauss_norm;
	double exponent;
	double weight;
	double sig_x=std_landmark[0];
	double sig_y=std_landmark[1];

	LandmarkObs cur_obs; 
	std::vector<LandmarkObs> transformed_observations;
	std::vector<LandmarkObs> predicted;

	for(int i= 0; i<pf.num_particles; i++)
	{	
		x_t=pf.particles[i].x;
		y_t=pf.particles[i].y;
		theta=pf.particles[i].theta;

		while (predicted.size() != 0) predicted.clear();
		while (transformed_observations.size() != 0) transformed_observations.clear();
		for (int j =0; i<observations.size(); j++)
		{	
			//get the current observation in map coordinates
			cur_obs.x = observations[j].x * cos(theta) - y * sin(theta) + x_t;
			cur_obs.y = observations[j].x; * sin(theta) - y * cos(theta) + y_t;
			transformed_observations.push_back(cur_obs)
		}
		//get the predictions

		/**watch out the way that the arguments are passed, see function definition : one must be dereferenced before passing*/
		pf.dataAssociation(predicted, transformed_observations);

		for (int j =0; i<observations.size(); j++){
			//calculate the likelihood of this observation
			gauss_norm= (1/(2 * M_PI * sig_x * sig_y));
			exponent= ((observations[j].x - predicted[j].x)*(observations[j].x - predicted[j].x))/(2 * sig_x*sig_x) + ((observations[j].y - predicted[j].y)*(observations[j].y - predicted[j].y))/(2 * sig_y*sig_y);
			weight= gauss_norm * exp(-exponent);
			//cummulative product of all the ovservations
			pf.particles[i].weight*= weight; 
		}
	}
}


void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
