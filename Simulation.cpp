/*
 * Simulation.cpp
 *
 *  Created on: Oct 12, 2016
 *      Author: ericwadkins
 */

#include "Simulation.h"

void start(time_t timer) {
    
    if (timer == -1) {
        timer = time(NULL);
    }
    std::cout << timer << std::endl;
    srand(timer);
    
	cv::namedWindow("Simulation", CV_WINDOW_AUTOSIZE);
    int width = 800;
    int height = 800;
    int pointCount = 500;
    int trailCount = 1000;
    int trailIndex = 0;
    int normalCount = 50;
    
    std::vector<std::pair<double, double> > positions;
    for (int i = 0; i < pointCount; i++) {
        double randX = rand() % (int) (width * 1.0) + width * 0.0;
        double randY = rand() % (int) (height * 1.0) + height * 0.0;
        double size = 0.0;
        for (int j = 0; j < normalCount; j++) {
            size += (double) rand() / RAND_MAX / normalCount;
        }
        size = (int) fmax(1, sqrt(fabs(0.5 - size)) * 10);
        for (int j = 0; j < size; j++) {
            positions.push_back(std::pair<double, double>(randX, randY));
        }
    }
    
    std::vector<std::pair<double, double> > trails(trailCount);

    
    std::vector<std::pair<double, double> > velocities;
    for (int i = 0; i < positions.size(); i++) {
        double randX = (double) rand() / RAND_MAX - 0.5;
        double randY = (double) rand() / RAND_MAX - 0.5;
        velocities.push_back(std::pair<double, double>(randX * 8, randY * 8));
    }
    
    std::vector<int> scales;
    for (int i = 0; i < positions.size(); i++) {
        scales.push_back(1);
    }

    long count = 0;
	while (true) {
        double mag = 0.1;
        double limit = 0.05;
        double dampen = 1;
        for (int i = 0; i < positions.size(); i++) {
            double x1 = positions[i].first;
            double y1 = positions[i].second;
            for (int j = 0; j < positions.size(); j++) {
                if (i != j) {
                    double x2 = positions[j].first;
                    double y2 = positions[j].second;
                    double dist = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
                    if (dist >= sqrt(scales[i] + scales[j])) {
                        double deltaX = (x2 - x1) * mag * (scales[i] + scales[j]) / fmax(1, pow(dist, 2));
                        double deltaY = (y2 - y1) * mag * (scales[i] + scales[j]) / fmax(1, pow(dist, 2));
                        velocities[i].first += (double) scales[j] / (scales[i] + scales[j]) * (fabs(deltaX) > limit ? deltaX : deltaX / dampen);
                        velocities[i].second += (double) scales[j] / (scales[i] + scales[j]) * (fabs(deltaY) > limit ? deltaY : deltaY / dampen);
                    }
                    else {
                        if (scales[j] > scales[i]) {
                            positions[i].first = positions[j].first;
                            positions[i].second = positions[j].second;
                        }
                        double weightedVelocityX = 0.0;
                        double weightedVelocityY = 0.0;
                        weightedVelocityX += velocities[i].first * sqrt(scales[i]) / sqrt(scales[i] + scales[j]);
                        weightedVelocityY += velocities[i].second * sqrt(scales[i]) / sqrt(scales[i] + scales[j]);
                        weightedVelocityX += velocities[j].first * sqrt(scales[j]) / sqrt(scales[i] + scales[j]);
                        weightedVelocityY += velocities[j].second * sqrt(scales[j]) / sqrt(scales[i] + scales[j]);
                        scales[i] += scales[j];
                        velocities[i].first = weightedVelocityX;
                        velocities[i].second = weightedVelocityY;
                        
                        positions.erase(positions.begin() + j);
                        velocities.erase(velocities.begin() + j);
                        scales.erase(scales.begin() + j);
                        if (j < i) {
                            i--;
                        }
                        j--;
                    }
                }
            }
        }
        
        for (int i = 0; i < positions.size(); i++) {
            positions[i].first = positions[i].first + velocities[i].first;
            positions[i].second = positions[i].second + velocities[i].second;
            trails[trailIndex % trailCount] = positions[i];
            trailIndex++;
        }
        
        cv::Mat display(cv::Size(width, height), CV_8UC3);
        display = cv::Scalar(0, 0, 0);
        for (int i = 0; i < trails.size(); i++) {
            cv::Scalar color = cv::Scalar(255, 255, 255);
            cv::circle(display, cv::Point(trails[i].first, trails[i].second),
                       1, color, -1);
        }
        for (int i = 0; i < positions.size(); i++) {
            cv::Scalar color = cv::Scalar(fmax(0, 255 - scales[i] * 5), 255, 255);
            if (scales[i] > 51) {
                color = cv::Scalar(0, fmax(0, 255 - (scales[i] - 50)), 255);
            }
            cv::circle(display, cv::Point(positions[i].first, positions[i].second),
                       sqrt(scales[i]), color, -1);
        }
        
        double total = 0.0;
        for (int i = 0; i < scales.size(); i++) {
            total += scales[i];
        }
        double shiftX = 0.0;
        double shiftY = 0.0;
        for (int i = 0; i < positions.size(); i++) {
            shiftX += positions[i].first * scales[i] / total;
            shiftY += positions[i].second * scales[i] / total;
        }
        for (int i = 0; i < positions.size(); i++) {
            double dist = sqrt(pow(positions[i].first - shiftX, 2) + pow(positions[i].second - shiftY, 2));
            if (dist > 3000) {
                positions.erase(positions.begin() + i);
                velocities.erase(velocities.begin() + i);
                scales.erase(scales.begin() + i);
            }
        }
        shiftX = width / 2 - shiftX;
        shiftY = height / 2 - shiftY;
        for (int i = 0; i < positions.size(); i++) {
            positions[i].first += shiftX;
            positions[i].second += shiftY;
        }
        /*if (shiftX > 5 || shiftY > 5) {
            for (int i = 0; i < trails.size(); i++) {
                trails[i].first = -1;
                trails[i].second = -1;
            }
        }*/

        
        if (count % 1 == 0) {
            // Display image
            cv::imshow("Simulation", display);

            // Handle keyboard input
            int key = cv::waitKey(1);
            /*if (key == 63232) {
                for (int i = 0; i < positions.size(); i++) {
                    positions[i].second += 5;
                }
            }
            if (key == 63233) {
                for (int i = 0; i < positions.size(); i++) {
                    positions[i].second -= 5;
                }
            }
            if (key == 63234) {
                for (int i = 0; i < positions.size(); i++) {
                    positions[i].first += 5;
                }
            }
            if (key == 63235) {
                for (int i = 0; i < positions.size(); i++) {
                    positions[i].first -= 5;
                }
            }*/
            if (key == 27) {
                exit(0);
            }
        }

		count++;
	}
}

int main(int argc, char** argv) {
    start(argc > 1 ? atoi(argv[1]) : -1);
}
