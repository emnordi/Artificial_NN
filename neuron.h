
#pragma once  
class neuron  
{  
private:  
    double input;  
    double localGradient;  
    double propagated_error;  
  
public:  
    neuron();  
    void SetInput(double input); //SETS INPUT TO NEURON  
    void SetLocalGrad(double x); //SETS LOCAL GRADIENT  
    void SetPError(double x); //SETS PROPAGATED ERROR  
  
    void resetInput(double input); //RESET VALUES OF NEURON  
  
    double getOutput();   
    double getInput();  
    double getLocalGrad();  
    double getPError();  
  
    void resetNeuron();  
};  