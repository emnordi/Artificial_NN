
#include "network.h"  
#include "neuron.h"  
#include "weight.h"  
#include "constants.h"  
#include <fstream>  
#include <string>  
#include <vector>  
#include <sstream>  
#include <random>  
#include <iostream>  
  
  
using namespace std;  
  
random_device rd;  
mt19937 gen(rd());  
uniform_real_distribution<> dis(-1, 1);  
  
double zeta_n = 0;  
  
  
Network::Network(int n_i, int n_h, int n_o, double z, double a)  
{  
  
    this->alpha = a;  
    this->zeta = z;  
  
    //BIAS NEURONS  
    this->input_layer.push_back(new neuron());  
    this->hidden_layer.push_back(new neuron());  
      
    //CREATES SETS OF NEURONS IN THE LAYERS  
    for (int i = 0; i < n_i; i++) {  
        this->input_layer.push_back(new neuron());  
    }  
  
    for (int i = 0; i < n_o; i++) {  
        this->output_layer.push_back(new neuron());  
    }  
  
    for (int i = 0; i < n_h; i++) {  
        this->hidden_layer.push_back(new neuron());  
    }  
  
    //CREATES THE WEIGHTS WITH RANDOM VALUES   
    for (int i = 0; i < (n_i + 1)*n_h; i++) {  
        Weight* w = new Weight;  
        w->SetWeight(dis(gen));  
        //w->SetWeight(0.5);  
        this->weights_1.push_back(w);  
    }  
  
    for (int i = 0; i < n_o * (n_h + 1); i++) {  
        Weight* w = new Weight;  
        w->SetWeight(dis(gen));  
        //w->SetWeight(0.5);  
        this->weights_2.push_back(w);  
    }  
  
    //SETS THE LINK WEIGHTS BETWEEN THE NEURONS   
    int cont = 0;  
    for (int i = 0; i < n_i+1; i++) {  
        for (int j = 1; j < n_h+1; j++) {  
            Weight* w = weights_1[cont];  
            w->SetLeft(input_layer[i]);  
            w->SetRight(hidden_layer[j]);  
            cont++;  
        }  
    }  
  
    cont = 0;  
    for (int i = 0; i < n_h+1; i++) {  
        for (int j = 0; j < n_o; j++) {  
            Weight* w = weights_2[cont];  
            w->SetLeft(hidden_layer[i]);  
            w->SetRight(output_layer[j]);  
            cont++;  
        }  
    }  
      
  
}  
  
vector<double> Network::forward(vector<double> input)  
{  
    this->resetNeurons();  
  
    //BIAS NEURONS SET TO 1  
    this->input_layer[0]->SetInput(1);  
    this->hidden_layer[0]->SetInput(1);  
  
    //FEEDFORWARD PROCCESS THROUGH THE LAYERS  
    //SET INPUT LAYERS  
    for (int i = 1; i <=input.size(); i++) {  
        this->getInputLayer()[i]->SetInput(input[i-1]);  
    }  
  
    //PASS VALUES TO HIDDEN LAYER  
    for (int i = 0; i < this->getWeights_1().size(); i++) {  
        Weight* w = this->getWeights_1()[i];  
        double r = (w->getLeft()->getInput()) * w->getWeight();  
        w->getRight()->SetInput(r);  
    }  
      
    //PASS VALUES TO OUTPUT LAYER  
    for (int i = 0; i < this->getWeights_2().size(); i++) {  
        Weight* w = this->getWeights_2()[i];  
        double x = i != 0 ? w->getLeft()->getOutput() : w->getLeft()->getInput();  
        double r = x * w->getWeight();  
        w->getRight()->SetInput(r);  
    }  
  
    vector<double> output;  
    //GET OUTPUT VALUES  
    for (int i = 0; i < this->getOutputLayer().size(); i++) {  
        output.push_back(this->getOutputLayer()[i]->getOutput());  
    }  
  
  
    /* 
    cout << "Weights 12" << endl; 
    for (int i = 0; i < this->getWeights_1().size(); i++) { 
        cout << this->getWeights_1()[i]->getWeight() << " "; 
    } 
    cout << endl; 
 
    cout << "Hidden" << endl; 
    for (int i = 0; i < this->getHiddenLayer().size(); i++) { 
        cout << this->getHiddenLayer()[i]->getOutput() << " "; 
    } 
    cout << endl; 
 
    cout << "Weights 2" << endl; 
    for (int i = 0; i < this->getWeights_2().size(); i++) { 
        cout << this->getWeights_2()[i]->getWeight() << " "; 
    } 
    cout << endl; 
 
    cout << "Output" << endl; 
    for (int i = 0; i < this->getOutputLayer().size(); i++) { 
        cout << this->getOutputLayer()[i]->getOutput() << " "; 
    } 
    cout << endl; 
 
    */  
  
      
  
    return output;  
}  
  
vector<neuron*> Network::getInputLayer()  
{  
    return input_layer;  
}  
  
vector<neuron*> Network::getHiddenLayer()  
{  
    return hidden_layer;  
}  
  
vector<neuron*> Network::getOutputLayer()  
{  
    return output_layer;  
}  
  
vector<Weight*> Network::getWeights_1()  
{  
    return weights_1;  
}  
  
vector<Weight*> Network::getWeights_2()  
{  
    return weights_2;  
}  
  
void Network::resetNeurons()  
{  
    //RESET INPUT VALUES FROM EACH NEURON  
    for (int i = 0; i < this->getInputLayer().size(); i++) {  
        this->getInputLayer()[i]->resetNeuron();  
    }  
    for (int i = 0; i < this->getHiddenLayer().size(); i++) {  
        this->getHiddenLayer()[i]->resetNeuron();  
    }  
    for (int i = 0; i < this->getOutputLayer().size(); i++) {  
        this->getOutputLayer()[i]->resetNeuron();  
    }  
}  
  
  
  
void Network::backpropagation(vector<double> errors, double total_errors_prev)  
{  
      
  
    //zeta_n = zeta *exp(20 * total_errors_prev);  
    //zeta_n = zeta;  
    alpha = 0;  
    zeta_n = zeta;  
    //Set local gradients  
  
    for (int i = 0; i < this->getOutputLayer().size(); i++) {  
        double output = this->getOutputLayer()[i]->getOutput();  
        this->getOutputLayer()[i]->SetLocalGrad(lambda * output * (1 - output) * errors[i]);  
    }  
  
    double error;  
    Weight* w;  
    double delta;  
    double grad;  
  
    //Set Deltas  
  
    for (int i = 0; i < this->getWeights_2().size(); i++) {  
        w = this->getWeights_2()[i];  
        grad = w->getRight()->getLocalGrad();  
        delta = grad * w->getLeft()->getOutput() * zeta_n;  
        w->SetDelta(delta + alpha * w->getDelta());  
    }  
      
    //Propagated Error  
  
    for (int i = 0; i < this->getWeights_2().size(); i++) {  
        w = this->getWeights_2()[i];  
        grad = w->getRight()->getLocalGrad();  
        w->getLeft()->SetPError(grad*w->getWeight());  
    }  
  
    //Local Grads   
    neuron* n;  
    double p_error, value;  
    for (int i = 0; i < this->getHiddenLayer().size(); i++) {  
        n = this->getHiddenLayer()[i];  
        p_error = n->getPError();  
        value = n->getOutput();   
        n->SetLocalGrad(zeta*value*(1-value)*p_error);  
    }  
  
    //Deltas W1  
      
    for (int i = 0; i < this->getWeights_1().size(); i++) {  
        w = this->getWeights_1()[i];  
        grad = w->getRight()->getLocalGrad();  
        delta = grad * w->getLeft()->getOutput() * zeta_n;  
        w->SetDelta(delta + alpha* w->getDelta());  
    }  
  
  
}  
  
void Network::updateWeights()  
{  
    for (int i = 0; i < this->getWeights_1().size(); i++) {  
        this->getWeights_1()[i]->update();  
    }  
  
    for (int i = 0; i < this->getWeights_2().size(); i++) {  
        this->getWeights_2()[i]->update();  
    }  
}  
  
void Network::inputWeights(string s)  
{  
    //INPUT WEIGHTS  
    ifstream file;  
    string line;  
    //file.open("C:/Users/Rodolfo Cuan/Documents/Essex/NN-Assigment-1/NN-Assigment-1/weights/weights.csv");  
    //file.open("C:/Users/Rodolfo Cuan/Documents/Essex/NN-Assigment/NN-Assigment/data_set.csv");  
    file.open(s);  
  
    getline(file, line);  
      
    stringstream lineStream(line);  
    string cell;  
  
    //READS WEIGHTS FROM FILE AND SET IT IN THE WEIGHTS  
  
    for (int i = 0; i < this->getWeights_1().size(); i++) {  
        getline(lineStream, cell, ',');  
        this->getWeights_1()[i]->SetWeight(stod(cell));  
    }  
  
    getline(file, line);  
    stringstream lineStream2(line);  
  
      
    for (int i = 0; i < this->getWeights_2().size(); i++) {  
        getline(lineStream2, cell, ',');  
        this->getWeights_2()[i]->SetWeight(stod(cell));  
    }  
  
    cout << "Weights Loaded" << endl;  
      
}  