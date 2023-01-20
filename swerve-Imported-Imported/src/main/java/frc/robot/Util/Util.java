// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

/** Add your docs here. */
public class Util {
   private Util(){}
    public static boolean isClose(double A, double B, double epsilion){
        return (A - epsilion <= B) && (A + epsilion >= B);
    }
    
}
