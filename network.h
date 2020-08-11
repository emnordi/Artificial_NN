
#pragma once  
#include <vector>  
#include <string>  
#include "weight.h"  
#include "neuron.h"  
//#include <math.h>  
  
using namespace std;  
  
class Network  
{  
private:  
  
    double zeta, alpha;  
  
    vector<neuron*> input_layer;  
    vector<neuron*> hidden_layer;  
    vector<neuron*> output_layer;  
  
    vector<Weight*> weights_1;  
    vector<Weight*> weights_2;  
  
  
      
public:  
    Network(int , int , int, double, double);  
  
    vector<double> forward(vector<double>); //FEEDFORWARD THE MODEL WITH GIVEN INPUTS  
      
    vector<neuron*> getInputLayer();  
    vector<neuron*> getHiddenLayer();  
    vector<neuron*> getOutputLayer();  
  
    vector<Weight*> getWeights_1();  
    vector<Weight*> getWeights_2();  
  
    void resetNeurons(); //EMPTY VALUES FOR THE NEURONS FOR NEXT LEARNING STEP  
  
    void backpropagation(vector<double>, double); // BACKPROPAGATION  
    void updateWeights(); //UPDATE WEIGHTS OF MODEL  
  
    void inputWeights(string); //INPUT WEIGHTS FROM FILE  
};  