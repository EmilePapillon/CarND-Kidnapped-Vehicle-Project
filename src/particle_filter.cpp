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

	this->num_particles = num_particles;
	for (int i = 0; i < this->num_particles; i++){
		//append to the particles vector
		Particle particle;
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
		particle.weight = 1;
		this->particles.push_back(particle);
	}

	this->is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	for(int i=0; i<this->num_particles; i++)
	{
		double std_x = std_pos[0];
		double std_y = std_pos[1];
		double std_theta = std_pos[2];


		this->particles[i].x = this->particles[i].x  +(velocity/yaw_rate)*(sin(this->particles[i].theta+yaw_rate*delta_t)-sin(this->particles[i].theta));
		this->particles[i].y = this->particles[i].y  + (velocity/yaw_rate)*(cos(this->particles[i].theta)-cos(this->particles[i].theta+yaw_rate*delta_t));
		this->particles[i].theta = this->particles[i].theta + yaw_rate*delta_t;

		//add noise
		default_random_engine gen;
		normal_distribution<double> dist_x(this->particles[i].x, std_x);
		normal_distribution<double> dist_y(this->particles[i].y, std_y);
		normal_distribution<double> dist_theta(this->particles[i].theta, std_theta);

		this->particles[i].x+=dist_x(gen);
		this->particles[i].x+=dist_y(gen);
		this->particles[i].x+=dist_theta(gen);
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	//Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	
	//use euclidian distance to find the nearest neigbor
	double euclidian_dist,smallest_dist;
	double x1, x2, y1, y2;
	int id;

	for (int i=0; i<observations.size(); i++){
		x1=observations[i].x;
		y1=observations[i].y;
		smallest_dist = numeric_limits<double>::max();

		for (int j =0; i<predicted.size(); i++){
			x2=predicted[j].x;
			y2=predicted[j].y;
			euclidian_dist=dist(x1,y1,x2,y2);

			if (euclidian_dist <  smallest_dist){
				smallest_dist= euclidian_dist;
				id=predicted[j].id;
			}
		}

		observations[i].id=id; 
	}

}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// Update the weights of each particle using a mult-variate Gaussian distribution. You can read
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

	double x_t, y_t, theta; 
	double gauss_norm, exponent, weight;
	double sig_x=std_landmark[0];
	double sig_y=std_landmark[1];
	LandmarkObs cur_obs; 
	std::vector<LandmarkObs> transformed_observations;

	std::vector<LandmarkObs> map_landmarks_as_LandMarkObs;
	for (int i=0; i<map_landmarks.landmark_list.size(); i++){
		double x = map_landmarks.landmark_list[i].x_f;
		double y = map_landmarks.landmark_list[i].y_f;
		double id = map_landmarks.landmark_list[i].id_i;
		LandmarkObs lm;
		lm.x=x;
		lm.y=y;
		lm.id=id;
		map_landmarks_as_LandMarkObs.push_back(lm);
	}

	/**
	loop through the particles vector and for each particle, transform the measurements into its coordinate 
	system, then associate each transformed measurement to a map landmark and then compute the likelihood of that measurement 
	actually corresponding to the landmark. Make a cummulative product for every observations.
	*/
	for(int i= 0; i<this->num_particles; i++)
	{	
		x_t=this->particles[i].x;
		y_t=this->particles[i].y;
		theta=this->particles[i].theta;

		//clear the transformed_observations vector
		while (transformed_observations.size() != 0) transformed_observations.clear();
		for (int j =0; i<observations.size(); j++)
		{	
			//get the current observation in map coordinates and save it to the vector
			cur_obs.x = observations[j].x * cos(theta) - observations[j].y * sin(theta) + x_t;
			cur_obs.y = observations[j].x * sin(theta) - observations[j].y * cos(theta) + y_t;
			transformed_observations.push_back(cur_obs);
		}

		//match the observations to map landmarks, save results in transformed_observations[i].id
		this->dataAssociation(map_landmarks_as_LandMarkObs, transformed_observations);

		//loop through the obersvations
		for (int j =0; i<transformed_observations.size(); j++){
			//get map landmark corresponding to each observation
			int id= transformed_observations[j].id;
			LandmarkObs ml;
			for (int k=0; k<map_landmarks_as_LandMarkObs.size(); k++){
				if (id == map_landmarks_as_LandMarkObs[k].id){
					ml=map_landmarks_as_LandMarkObs[k];
				}
			}
			//calculate the likelihood of this observation being the associated landmark, using a multivariate normal distribution
			gauss_norm= (1/(2 * M_PI * sig_x * sig_y));
			exponent= ((observations[j].x - ml.x)*(observations[j].x - ml.x))/(2 * sig_x*sig_x) + ((observations[j].y - ml.y)*(observations[j].y - ml.y))/(2 * sig_y*sig_y);
			weight= gauss_norm * exp(-exponent);
			//cummulative product of all the observations
			this->particles[i].weight*= weight; 
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
