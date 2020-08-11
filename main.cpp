
#include <iostream>  
#include <fstream>  
#include <string>  
#include <vector>  
#include <sstream>  
#include <algorithm>  
#include "network.h"  
#include "neuron.h"  
#include <random>  
#include <iomanip>  
  
  
using namespace std;  
  
  
const double validation_size = 0.15;  
const double test_size = 0.15;  
const double train_size = 0.7;  
  
struct data_sample  
{  
    double x1, x2;  
    double y1, y2;  
  
};  
  
vector<data_sample> slice(vector<data_sample>::iterator begin, vector<data_sample>::iterator end)  
{  
    //SLICING SLICING VECTOR  
    vector<data_sample> output;  
    for (vector<data_sample>::iterator i = begin; i != end; ++i)  
    {  
        output.push_back(*i);  
    }  
    return output;  
}  
  
void training(int input_n, int hidden_n, int output_n,  int epoch, int min_error, double z, double a, string weights_name, string graph_data) {  
      
    //TRAINING NN  
  
    vector<data_sample> data_set;  
    vector<data_sample> val_set;  
    vector<data_sample> test_set;  
    vector<data_sample> train_set;  
  
    ifstream file;  
    string line;  
    //file.open("M:/NN-Assigment-1/NN-Assigment-1/data_nn_norm-1-1.csv");  
    file.open("C:/Users/Rodolfo Cuan/Documents/Essex/NN-Assigment-1/data_nn_norm_1-3.csv");  
  
    //READ DATA FROM CSV AND STORES IT IN DATA STRUCTURE  
  
    while (getline(file, line))  
    {  
        stringstream lineStream(line);  
        string cell;  
  
        data_sample data;  
  
        getline(lineStream, cell, ',');  
        data.x1 = stod(cell);  
        getline(lineStream, cell, ',');  
        data.x2 = stod(cell);  
        getline(lineStream, cell, ',');  
        data.y1 = stod(cell);  
        getline(lineStream, cell, ',');  
        data.y2 = stod(cell);  
  
        data_set.push_back(data);  
  
    }  
  
    file.close();  
  
  
    // SHUFFLES AND SLICE DATA  
  
    random_shuffle(data_set.begin(), data_set.end());  
  
    vector<data_sample>::iterator begin_test = data_set.begin() + ceil(data_set.size() * train_size);  
    vector<data_sample>::iterator begin_val = begin_test + ceil(data_set.size() * test_size);  
  
    train_set = slice(data_set.begin(), begin_test);  
    test_set = slice(begin_test, begin_val);  
    val_set = slice(begin_val, data_set.end());  
  
    //CREATES NN  
  
    Network net(input_n, hidden_n, output_n, z, a);  
  
    string path = "C:/Users/Rodolfo Cuan/Documents/Essex/NN-Assigment-1/weights1-0.7z-0.1a-bravo.csv";  
  
    //INPUT WEIGHTS IF NECESARY  
    //net.inputWeights(path);  
  
    //PARAMETERS  
  
    vector<double> input;  
    vector<double> output;  
    vector<double> errors;  
    vector<double> minWeights1;  
    vector<double> minWeights2;  
    double minError = 1;  
  
  
    double error = 0;  
  
    vector<double> graphing_train;  
    vector<double> graphing_validation;  
  
  
    double total_errors = 0;  
    double total_total_errors = 0;  
    double total_errors_prev = 0;  
    int epoch_c = 0;  
  
  
    double e = 1;  
    double e_total = 1;  
  
    //STOPPING CRITERIA EPOCH OR WHEN REACHED AN DETERMINE ERROR  
    for (int j = 0; j < epoch; j++) {//while(e_total  > 0.0005){  
  
        total_total_errors = 0;  
  
        for (int i = 0; i < train_set.size(); i++) {  
  
            errors.clear();  
  
            total_errors = 0;  
  
            //GETS INPUT DATA  
            input = { train_set[i].x1 , train_set[i].x2 };  
  
            //GETS OUTPUT  
            output = net.forward(input);  
  
            //ERROR CALCULATIONS  
            error = train_set[i].y1 - output[0];  
            total_errors += pow(pow(error, 2),0.5);  
            errors.push_back(error);  
            error = train_set[i].y2 - output[1];  
            errors.push_back(error);  
            total_errors += pow(pow(error, 2), 0.5);  
            total_errors /= 2;  
            total_total_errors += total_errors;  
  
            //GENERATES BACKPROPAGATION  
            net.backpropagation(errors, total_errors_prev);  
  
            //UPDATES WEIGHTS  
            net.updateWeights();  
        }  
  
        //ERROR CALCULATION AND PRINTING  
  
        total_total_errors /= train_set.size();  
        total_errors_prev = total_total_errors;  
  
        cout << "epoch " << epoch_c << ":" << total_total_errors;  
  
        graphing_train.push_back(total_total_errors);  
  
        //SHUFFLE TRAINING DATASET  
        random_shuffle(train_set.begin(), train_set.end());  
  
        //VALDATION CALULATION  
        e_total = 0;  
        for (int i = 0; i < val_set.size(); i++) {  
            vector<double> v = { val_set[i].x1, val_set[i].x2 };  
            vector<double> y = net.forward(v);  
  
            e = 0;  
  
            e += pow(pow(y[0] - val_set[i].y1, 2),0.5);  
            e += pow(pow(y[1] - val_set[i].y2, 2),0.5);  
            e /= 2;  
            e_total += e;  
  
        }  
  
        e_total /= val_set.size();  
          
  
        cout << " Validation Acc: " << e_total << endl;  
        graphing_validation.push_back(e_total);  
  
        //STOPPING CRITERIA  
        if (e_total < min_error) { break; }  
  
        epoch_c++;  
  
        //SAVES THE WEIGHTS OF THE LEAST ERROR OBTAINED  
  
        if (e_total < minError) {  
            minError = e_total;  
            minWeights1.clear();  
            minWeights2.clear();  
            for (int i = 0; i < net.getWeights_1().size(); i++) {  
                minWeights1.push_back(net.getWeights_1()[i]->getWeight());  
            }  
            for (int i = 0; i < net.getWeights_2().size(); i++) {  
                minWeights2.push_back(net.getWeights_2()[i]->getWeight());  
            }  
        }  
  
  
    }  
  
    //TESTING  
  
    e_total = 0;  
    for (int i = 0; i < test_set.size(); i++) {  
        vector<double> v = { test_set[i].x1, test_set[i].x2 };  
        vector<double> y = net.forward(v);  
  
        e = 0;  
  
        e += pow(y[0] - test_set[i].y1, 2);  
        e += pow(y[1] - test_set[i].y2, 2);  
        e /= 2;  
        e = pow(e, 0.5);  
        e_total += e;  
  
    }  
  
    e_total /= test_set.size();  
  
    cout << " Test Acc: " << e_total << endl;  
  
    //CREATES FILE WITH ERRORS AND WEIGHTS  
  
    ofstream outputFileg;  
    outputFileg.open("C:/Users/Rodolfo Cuan/Documents/Essex/NN-Assigment-1/"+graph_data + ".csv");  
    for (int i = 0; i < graphing_validation.size(); i++) {  
        outputFileg << graphing_validation[i] << "," << graphing_train[i] << endl;  
    }  
  
    outputFileg.close();  
  
    ofstream outputFile;  
    outputFile.open("C:/Users/Rodolfo Cuan/Documents/Essex/NN-Assigment-1/" + weights_name + ".csv");  
    for (int i = 0; i < minWeights1.size(); i++) {  
        outputFile << minWeights1[i] << ",";  
    }  
    outputFile << endl;  
    for (int i = 0; i < minWeights2.size(); i++) {  
        outputFile << minWeights2[i] << ",";  
    }  
    outputFile.close();  
  
}  
  
  
void testing(int input_n, int hidden_n, int output_n, double z, double a,  string s) {  
  
    //FUNCTION TO TEST WEIGHTS AND THE OUTPUT THEY GIVE  
  
    Network net(input_n, hidden_n, output_n,z,a);  
  
    //NAME FOR WEIGHT FILE  
    string path = "C:/Users/Rodolfo Cuan/Documents/Essex/NN-Assigment-1/"+s+".csv";  
  
    net.inputWeights(path);  
  
    vector<double> output;  
  
    double x, y;  
    while (1) {  
        cin >> x >> y;  
        vector<double> input = { x,y };  
  
        output = net.forward(input);  
  
        cout << output[0] <<" "<< output[1] << endl;  
  
    }  
}  
  
  
  
int main()  
{  
  
    int hid_nn = 3;  
    double z = 0.6;  
    double a = 0.8;  
    string name = "bravo3.1";  
  
  
    training(2, 3, 2, 100, 0.01, z, a, "weights_r-n"+ to_string(0.1)+"-"+ to_string(z)+"z-" + to_string(a) + "a-" + name, "graph_report-NM-n"+to_string(2)+"-"+ to_string(z)+"z-" + to_string(a) + "a-" + name);  
      
  
      
    return 0;  
}  