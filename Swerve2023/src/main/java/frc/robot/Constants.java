// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */


public final class Constants {

    public static final class DriveConstants {

        //conversion factors
        public static final double kpinonTeeth = 14;
        public static final double kwheeldiameter = Units.inchesToMeters(3);   // 3in 
        public static final double kdriveReduction = (45*22)/(kpinonTeeth*15);  //(16.0 / 32.0) * (15.0 / 45.0);
        public static final double positionConversionFactor = (Math.PI * kwheeldiameter) / kdriveReduction;
        public static final double ksteeringReduction = (10.0 / 30.0 ) * (18.0 / 96.0);
        public static final double analogPositionConversionFactor = (2 * Math.PI / 3.3);
        //drive motor pid
        public static final double kDriveP = 0.01;  
        public static final double kDriveI = 0;
        public static final double kDriveD = 0.6;//0.1;
        public static final double kDriveF = 0.21;//0.2;
        //turning motor pid
        public static final double kTurnP = 0.7;//0.8;
        public static final double kTurnI = 0;//0.00;
        public static final double kTurnD = 0.02;//0.02;
        public static final double kTurnF = 0;

        //Drive Motor Constants
        public static final double kDriveEncoderPositionFactor = positionConversionFactor;
        public static final double kDriveEncoderVelocityFactor = positionConversionFactor / 60.0;
        public static final double kDriveMinOutput = -1;
        public static final double kDriveMaxOutput = 1;
        public static final IdleMode kDriveMotorIdleMode = IdleMode.kBrake;    
        public static final int kDriveMotorCurrentLimit = 50; // amps
        //Turn Motor Constants
        public static final double kTurnEncoderPositionFactor = 2 * Math.PI;
        public static final double kTurnEncoderVelocityFactor = 2 * Math.PI;
        public static final boolean kTurnEncoderInverted = true;
        public static final double kTurnEncoderPositionPIDMinInput = 0;
        public static final double kTurnEncoderPositionPIDMaxInput = 2*Math.PI;
        public static final double kTurnMinOutput = -1;
        public static final double kTurnMaxOutput = 1;
        public static final IdleMode kTurnMotorIdleMode = IdleMode.kBrake;
        // public static final IdleMode kTurnMotorIdleMode = IdleMode.kCoast;
        public static final int kTurnMotorCurrentLimit = 20; // amps

        public static double kFrontLeftOffset = -Math.PI / 2;  
        public static double kBackLeftOffset = Math.PI;
        public static double kFrontRightOffset = 0;
        public static double kBackRightOffset = Math.PI / 2;

        public static double k_LeftEvasiveX = 0.0635;
        public static double k_LeftEvasiveY = 0.6477;
        public static double k_RightEvasiveX = 0.6477;
        public static double k_RightEvasiveY = -0.0635;
        

        public static double kMaxSpeedMetersPerSecond = 4.8;
        public static int kNavXAdjustment = 0;

        //motor assignments
        public static final int kFrontLeftTurningID = 13;    
        public static final int kFrontLeftDriveID = 14;
        public static final int kBackLeftTurningID = 16;
        public static final int kBackLeftDriveID = 15;

        public static final boolean kFrontLeftDriveMotorInverted = false;
        public static final boolean kFrontRightDriveMotorInverted = false;
        public static final boolean kBackLeftDriveMotorInverted = false;
        public static final boolean kBackRightDriveMotorInverted = false;
        
        public static final int kFrontRightTurningID = 6;
        public static final int kFrontRightDriveID = 5;
        public static final int kBackRightTurningID = 3;
        public static final int kBackRightDriveID = 4;

        public static final int kFrontLeftDriveAnalogPort = 6;
        public static final int kFrontRightDriveAnalogPort = 7;
        public static final int kBackLeftDriveAnalogPort = 8;
        public static final int kBackRightDriveAnalogPort = 9;
        
        public static final double kMaxAngularSpeed = 4*Math.PI;
        public static final boolean kGyroReversed = false;

        public static final double kTrackWidth = Units.inchesToMeters(18.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(22.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); 

    }

    public static final class BlinkinConstants{
      public static final double kSkyBlue = .83;
      public static final double kRed = 0.61;
      public static final double kRedStrobe = -.11;
      public static final double kGreen = .77;
      public static final double kFireLarge = -.57;
      public static final double kTippedFront = .61;
      public static final double kTippedFrontFar = .57;
      public static final double kTippedBack = .91;
      public static final double kTippedBackFar = .87;
      public static final double kBalanced = .77;
      public static final double kYellow = 0.69;
      public static final double kPurple = .89;
      public static final double kColor1Chaser = 0.01;
      public static final double kColor2Chaser = 0.21;
      public static final double kColor1Blink = -.01;
      public static final double kColor2Blink = .19;
    }

    public static final class LimeLightConstants{
      public static final double[] kTarget1Constants = {1, 1, 0};
      public static final double[] kTarget2Constants = {2, 2, 0};
      public static final double[] kTarget3Constants = {3, 3, 0};
      public static final double[] kTarget4Constants = {4, 4, 0};
      public static final double[] kTarget5Constants = {5, 5, 180};
      public static final double[] kTarget6Constants = {6, 6, 180};
      public static final double[] kTarget7Constants = {7, 7, 180};
      public static final double[] kTarget8Constants = {8, 8, 180};
    }
    
    public static final class AutoConstants {
        // public static final double kMaxSpeedMetersPerSecond = DriveConstants.kMaxSpeedMetersPerSecond;
        // public static final double kMaxAccelerationMetersPerSecondSquared = DriveConstants.kMaxSpeedMetersPerSecond;
        public static final double kMaxSpeedMetersPerSecond = 4.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 4.0;
     
        public static final double kMaxAngularSpeedRadiansPerSecond = 4*Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 4*Math.PI;
    
        public static final double kPXController = 2;
        public static final double kPYController = 2;
        public static final double kPThetaController = 2;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }

      public static final class IntakeConstants{
        public static final int kIntakeMotorCanID = 19;
        public static final double kIntakeForward = 500;
        public static final double kIntakeForwardPower = 1;
      }

      public static final class ArmConstants{
        public static final int kArmMotorCanID = 18;
        public static final double kArmP = 0.00;
        public static final double kArmI = 0;
        public static final double kArmD = 0.;
        public static final double kArmF = 0.0;
        public static final double kArmForward = .3;
        public static final double kArmOffset = -.487;
        public static final double kPowerLimit = .2;
      }

}
