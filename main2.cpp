
#include <fstream>   
#include <iostream>  
#include <vector>  
#include "aria.h"  
  
#include "network.h"  
#include "neuron.h"  
  
  
#include <fstream>   
#include <iostream>  
using namespace std;  
  
  
  
int main(int argc, char** argv)  
{  
  
    Aria::init();  
    ArRobot robot;  
  
    ArArgumentParser argParser(&argc, argv);  
    argParser.loadDefaultArguments();  
  
    ArRobotConnector robotConnector(&argParser, &robot);  
  
    if (robotConnector.connectRobot())  
        std::cout << "Robot connected!" << std::endl;  
  
    robot.runAsync(false);  
    robot.lock();  
    robot.enableMotors();  
    robot.unlock();  
  
  
    ArSensorReading* sonarSensor[8];  
  
    int sonarRange[8];  
    double sonarRangeMin[8] = { 5000,5000,5000,5000,5000,5000,5000,5000 };  
  
    int cont = 0;  
    const float target = 450;  
    double acc_val = 0, left_sp = 0, right_sp = 0;  
  
  
      
    ofstream file;  
  
    ArUtil::sleep(2000);  
  
    //Creates Network  
    Network net(2,3, 2, 0, 0);  
  
    //string path = "M:/NN-Assigment-Final/RobotNN/RobotNN/weightsweights1-0.01z-0.01a-beta.csv";  
    //string path = "M:/NN-Assigment-Final/RobotNN/RobotNN/weights-8hn-2a6.500000z-charlie.csv";  
    string path = "M:/NN-Assigment-Final/RobotNN/RobotNN/weights-n3-0.600000z-0.800000a-bravo3.csv";  
  
    //Input Weights  
    net.inputWeights(path);  
  
    //Values to Normalize and Denormalize  
    double max_left_i = 4715.32;  
    double min_left_i = 227.785;  
    double max_right_i = 4318.88;  
    double min_right_i = 547.331;  
      
    double max_left = 249.564;  
    double min_left = 103.007;  
    double max_right = 300;  
    double min_right = 67.7208;  
  
      
    while (true)  
    {  
  
        //Read sensor Readings  
        for (int i = 0; i < 8; i++)  
        {  
            sonarSensor[i] = robot.getSonarReading(i);  
            sonarRange[i] = sonarSensor[i]->getRange();  
  
            sonarRangeMin[i] = (sonarRangeMin[i] < sonarRange[i]) ? sonarRangeMin[i] : sonarRange[i];  
            //sonarRangeMin[i] = sonarRange[i];  
              
  
        }  
  
        cont++;  
  
          
        if (cont == 1) {  
            cont = 0;  
  
                //Limit sensor readings and normalize  
                vector<double> output;  
  
  
                cout << sonarRangeMin[0] << " " << sonarRangeMin[1] << " ";  
  
                sonarRangeMin[0] = sonarRangeMin[0] > sonarRangeMin[1] ? sonarRangeMin[1] : sonarRangeMin[0];  
                sonarRangeMin[1] = sonarRangeMin[2] > sonarRangeMin[3] ? sonarRangeMin[3] : sonarRangeMin[2];  
              
  
                file.open("t2.csv", ios::app);  
  
                file << "," << sonarRangeMin[0];  
                file << "," << sonarRangeMin[1];  
  
                file << "\n";  
  
                file.close();  
              
                sonarRangeMin[0] = sonarRangeMin[0] > max_left_i ? max_left_i : sonarRangeMin[0];  
                sonarRangeMin[0] = sonarRangeMin[0] < min_left_i ? min_left_i : sonarRangeMin[0];  
  
                sonarRangeMin[1] = sonarRangeMin[1] > max_right_i ? max_right_i : sonarRangeMin[1];  
                sonarRangeMin[1] = sonarRangeMin[1] < min_right_i ? min_right_i : sonarRangeMin[1];  
  
                cout << sonarRangeMin[0] << " " << sonarRangeMin[1];  
  
                sonarRangeMin[0] = (sonarRangeMin[0] - min_left_i) / (max_left_i - min_left_i);  
                sonarRangeMin[1] = (sonarRangeMin[1] - min_right_i) / (max_right_i - min_right_i);  
  
                //2811 707 98.457 261.608  
  
  
                //cout << sonarRangeMin[0] << " " << sonarRangeMin[1] << " ";  
  
  
                vector<double> input = { sonarRangeMin[0],sonarRangeMin[1] };  
  
                //Feedforward  
                output = net.forward(input);  
  
  
                //Denormalize  
                left_sp = output[0] * (max_left - min_left) + min_left;  
                right_sp = output[1] * (max_right - min_right) + min_right;  
  
  
  
                //Move Robot  
  
                cout << " " << left_sp << " " << right_sp << endl;  
  
                robot.setVel2(left_sp, right_sp);  
  
  
                  
                  
  
                for (int i = 0; i < 8; i++)  
                {  
                      
  
                    sonarRangeMin[i] = 5000;  
  
  
                }  
              
              
  
  
        }  
  
        //ArUtil::sleep(100);  
    }  
  
    robot.lock();  
    robot.stop();  
    robot.unlock();  
  
    Aria::exit();  
  
    return 0;  
}  