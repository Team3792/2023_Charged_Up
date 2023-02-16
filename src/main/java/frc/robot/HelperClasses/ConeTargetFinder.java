// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.HelperClasses;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

/** Add your docs here. */


public class ConeTargetFinder {

    Pose2d[][] dropLocations; //These should be in meters, fill them in later in constants
    double [] dropboundaries;


    public ConeTargetFinder(){
        if(DriverStation.getAlliance() == DriverStation.Alliance.Red){
        dropLocations = Constants.FieldLayoutConstants.kRedCubeDropLocations;
        }   else{
            dropLocations = Constants.FieldLayoutConstants.kBlueCubeDropLocations;
        }
        findBoundaries();
    }

    private void findBoundaries (){
        //This loops through all but the last location and defines the boundary as the midpoint between i and i+1
        for(int i = 0; i < dropLocations.length - 1; i++){
            dropboundaries[i] = (dropLocations[i][0].getY() + dropLocations[i+1][0].getY())/2;
            //Use the ground nodes y value to get reference y locations
            //the rhs takes the average of 2 consecutive locations
            //[i].getY() is the y-coordinate (what separates the zones/what will define the zones) of the i-th drop location
        }
    }

    //This function is given the y-coordinate of the robot and returns the zone the 
    //To Do - Decide on standards for these namings
    public int getZone(double robotYLocation){
        int allianceRelativeZoneKey = 5;
        //This loops through each elements. The first time the robot location is less than the boundary, it returns
        //ID for the zone (which is conveniently the same as its array index)
        //If none of the boundaries are greater than the location, then it return the greatest farthest region.
        for(int i = 0; i < dropboundaries.length; i++){
            if(robotYLocation< dropboundaries[i]){
                allianceRelativeZoneKey = i;
                break;
            }
        }
        return allianceRelativeZoneKey;
    }

    public Pose2d getTarget(int elevatorHeight, double robotYLocation){
        int zone = getZone(robotYLocation);
        //Depending on which alliance we are on, access a different field layout.
        Pose2d target = dropLocations[zone][elevatorHeight];

        return target; 
    }
}
