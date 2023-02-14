// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//This isn't needed any more since the vision code already has it in it, but we should keep it here just in case.
package frc.robot.HelperClasses;

import frc.robot.Constants;
import java.lang.Math;

/** Add your docs here. */
public class RobotLayout {
    //All positisions relative to lower left corner of robot
    //first number, signed x-value, second number, signed y-value
    double[] camera1RelativeLocation = Constants.RobotLayoutConstants.kCamera1Location;
    double[] turretRelativeLocation = Constants.RobotLayoutConstants.kTurretLocation;
    double[] cameraToTurretVector = {
        turretRelativeLocation[0] - camera1RelativeLocation[0],
        turretRelativeLocation[1] - camera1RelativeLocation[1]
    };

    double[] getTurretLocation(double[] cameraAbsolutePosition, double theta){
        double[] rotatedCamerToTurretVector = rotate(theta, cameraToTurretVector);
        double[] turretAbsoluteLocation = 
        {
            cameraAbsolutePosition[0] + rotatedCamerToTurretVector[0],
            cameraAbsolutePosition[1] + rotatedCamerToTurretVector[1]
        };

        return turretAbsoluteLocation;
    }

    double[] rotate(double theta, double[] vector){
        /*
         * Rotation matrix;
         * -sin     cos
         * cos      sin
         */

    double[] rotatedVector = 
    {
        vector[1]*Math.cos(theta) + vector[0] * (-Math.sin(theta)),
        vector[0]*Math.cos(theta) + vector[1] * (Math.sin(theta))

    };

    return rotatedVector;
        
    }
    


}
