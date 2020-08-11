
#include "neuron.h"  
#include <math.h>  
#include "constants.h"  
  
neuron::neuron()  
{  
    input = 0;  
    propagated_error = 0;  
    localGradient = 0;  
}  
  
void neuron::SetInput(double x)  
{  
    input += x;  
}  
  
void neuron::SetLocalGrad(double x)  
{  
    this->localGradient = x;  
}  
  
void neuron::SetPError(double x)  
{  
    this->propagated_error += x;  
}  
  
void neuron::resetInput(double input)  
{  
    this->input = input;  
}  
  
double neuron::getOutput()  
{  
    //SIGMOID FUNCTION  
    return (1 / (1 + exp(-lambda * (this->input))));  
}  
  
double neuron::getInput()  
{  
    return input;  
}  
  
double neuron::getLocalGrad()  
{  
    return this->localGradient;  
}  
  
double neuron::getPError()  
{  
    return this->propagated_error;  
}  
  
void neuron::resetNeuron()  
{  
    input = 0;  
    propagated_error = 0;  
}  