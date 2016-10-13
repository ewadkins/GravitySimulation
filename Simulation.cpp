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
    double scale = 1;
    double offsetX = 0.0;
    double offsetY = 0.0;
    int speed = 1;
    
    int radiusNormalization = 1;
    int sizeNormalization = 50;
    
    std::vector<std::pair<double, double> > positions;
    std::vector<std::pair<double, double> > trails(trailCount);
    std::vector<std::pair<double, double> > velocities;
    std::vector<double> scales;
    
    for (int i = 0; i < pointCount; i++) {
        //double x = rand() % (int) (width * 1.0) + width * 0.0;
        //double y = rand() % (int) (height * 1.0) + height * 0.0;
        double r = 0.0;
        for (int j = 0; j < radiusNormalization; j++) {
            r += (double) (rand() % (int) (fmin(width, height) / 2)) / radiusNormalization;
        }
        double theta = (double) rand() / RAND_MAX * 2 * M_PI;
        double x = r * sin(theta) + width / 2;
        double y = r * cos(theta) + height / 2;
        double size = 0.0;
        for (int j = 0; j < sizeNormalization; j++) {
            size += (double) rand() / RAND_MAX / sizeNormalization;
        }
        size = (int) fmax(1, sqrt(fabs(0.5 - size)) * 10);
        for (int j = 0; j < size; j++) {
            positions.push_back(std::pair<double, double>(x, y));
            //velocities.push_back(std::pair<double, double>(sin(theta + M_PI / 2) / fmax(5, sqrt(r)) * 10 * (rand() % 2 == 0 ? -1 : 1), cos(theta + M_PI / 2) / fmax(5, sqrt(r)) * 10 * (rand() % 2 == 0 ? -1 : 1)));
        }
    }
    
    for (int i = 0; i < positions.size(); i++) {
        double randX = (double) rand() / RAND_MAX - 0.5;
        double randY = (double) rand() / RAND_MAX - 0.5;
        velocities.push_back(std::pair<double, double>(randX * 12, randY * 12));
    }
    
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
            cv::circle(display, cv::Point((trails[i].first - width / 2) / scale + width / 2 + offsetX, (trails[i].second - height / 2) / scale + height / 2 + offsetY),
                       (int) (sqrt(1 / scale) - 0.1), color, -1);
        }
        for (int i = 0; i < positions.size(); i++) {
            cv::Scalar color = cv::Scalar(fmax(0, 255 - scales[i] * 5), 255, 255);
            if (scales[i] > 51) {
                color = cv::Scalar(0, fmax(0, 255 - (scales[i] - 50)), 255);
            }
            cv::circle(display, cv::Point((positions[i].first - width / 2) / scale + width / 2 + offsetX, (positions[i].second - height / 2) / scale + height / 2 + offsetY),
                       sqrt(scales[i]) / scale, color, -1);
        }
        
        double avgTotal = 0.0;
        for (int i = 0; i < scales.size(); i++) {
            avgTotal += scales[i];
        }
        double avgX = 0.0;
        double avgY = 0.0;
        for (int i = 0; i < positions.size(); i++) {
            avgX += positions[i].first * scales[i] / avgTotal;
            avgY += positions[i].second * scales[i] / avgTotal;
        }
        
        // Remove if too far away
        /*for (int i = 0; i < positions.size(); i++) {
            double dist = sqrt(pow(positions[i].first - avgX, 2) + pow(positions[i].second - avgY, 2));
            if (dist > fmin(width, height) * 4) {
                positions.erase(positions.begin() + i);
                velocities.erase(velocities.begin() + i);
                scales.erase(scales.begin() + i);
            }
            
        }*/
        
        // Adjust frame of reference
        double shiftX = width / 2 - avgX;
        double shiftY = height / 2 - avgY;
        for (int i = 0; i < positions.size(); i++) {
            positions[i].first += shiftX;
            positions[i].second += shiftY;
        }
        for (int i = 0; i < trails.size(); i++) {
            trails[i].first += shiftX;
            trails[i].second += shiftY;
        }
        for (int i = 0; i < velocities.size(); i++) {
            velocities[i].first += shiftX;
            velocities[i].second += shiftY;
        }

        
        if (count % speed == 0) {
            // Display image
            cv::imshow("Simulation", display);

            // Handle keyboard input
            int key = cv::waitKey(1);
            if (key == 63232) { // up
                offsetY += 5;
            }
            if (key == 63233) { // down
                offsetY -= 5;
            }
            if (key == 63234) { // left
                offsetX += 5;
            }
            if (key == 63235) { // right
                offsetX -= 5;
            }
            else if (key == 61) { // plus
                scale *= 3.0/4;
                scale = fmax(scale, pow(3.0/4, 5));
            }
            else if (key == 45) { // minus
                scale *= 4.0/3;
            }
            else if (key == 115) { // s (slow)
                speed -= 1;
                speed = fmax(1, speed);
            }
            else if (key == 102) { // f (fast)
                speed += 1;
            }
            else if (key == 27) { // esc
                exit(0);
            }
            else if (key != -1) {
                //std::cout << key << std::endl;
            }
        }

		count++;
	}
}

int main(int argc, char** argv) {
    start(argc > 1 ? atoi(argv[1]) : -1);
}
