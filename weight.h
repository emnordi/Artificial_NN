
#pragma once  
#include "neuron.h"  
  
  
class Weight  
{  
private:  
    double w, delta;  
    neuron* left;  
    neuron* right;  
  
public:  
    Weight();  
    void SetWeight(double w); //SET WEIGHT  
    void SetDelta(double d); //SET WEIGHT DELTA  
    void SetLeft(neuron* l); //SET THE NEURON IN THE LEFT SIDE OF THE WEIGHT  
    void SetRight(neuron* r); //SET THE NEURON IN THE RIGHT SIDE OF THE WEIGHT  
  
    double getWeight();  
    double getDelta();  
    neuron* getLeft();  
    neuron* getRight();  
    void update();  
};  