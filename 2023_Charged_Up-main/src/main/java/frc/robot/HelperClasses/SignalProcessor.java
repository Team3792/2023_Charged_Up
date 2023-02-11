package frc.robot.HelperClasses;

import java.lang.Math;


public class SignalProcessor{

double maxOutput, deadband;
int curveType;


    public SignalProcessor(double maxOutput, double deadband, int curveType){
        //Parameter explanation:

        /*All signals come in from -1 to 1 (from joystick) where 0 will always be 0 when mapped
        (we can change this if we want)
        maxOutput it the output we want when the signal is 1
        deadband is, of course, the deadband, and is applied + and -.
        curve type is what curve we apply to the signal:

        0 - linear
        1 - sigmoid
        2 - power curve

        */

        this.maxOutput = maxOutput;
        this.deadband = deadband;
        this.curveType = curveType;

    }

    private double linearMap(double input){
        return input*maxOutput;
    }

    private double sigmoidMap(double input){
        return maxOutput*(2/(1+Math.pow(0.135335283237, input))-1)*(1.3130352855);
    }

    private double powerMap(double input){
        return maxOutput*input*input*input;
    }

    private double deadbandCalculate(double input){
        if(Math.abs(input) <= deadband){
            return 0;
        }else{
            return input;
        }
    }
    
    public double getOutput(double input){

        double deadbandedInput = deadbandCalculate(input);
        double output = 0; 

        switch(curveType){
            case 0:
                output =  linearMap(deadbandedInput);

            case 1:
                output =  sigmoidMap(deadbandedInput);

            case 2:
                output = powerMap(deadbandedInput);

        }

        return output;

    }

}