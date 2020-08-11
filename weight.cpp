
#include "weight.h"  
#include <stdlib.h>  
#include <stdio.h>   
  
  
Weight::Weight()  
{  
    double w = 0;  
    double delta = 0;  
    neuron* left = NULL;  
    neuron* right = NULL;  
}  
  
void Weight::SetWeight(double w)  
{  
    this->w = w;  
}  
  
void Weight::SetDelta(double d)  
{  
    this->delta = d;  
}  
  
void Weight::SetLeft(neuron* l)  
{  
    this->left = l;  
}  
  
void Weight::SetRight(neuron* r)  
{  
    this->right = r;  
}  
  
double Weight::getWeight()  
{  
    return w;  
}  
  
double Weight::getDelta()  
{  
    return delta;  
}  
  
neuron* Weight::getLeft()  
{  
    return left;  
}  
  
neuron* Weight::getRight()  
{  
    return right;  
}  
  
void Weight::update()  
{  
    //UPDATE WEIGHT  
    this->w += this->delta;  
}  