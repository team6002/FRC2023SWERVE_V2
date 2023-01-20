// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.fasterxml.jackson.databind.deser.std.UntypedObjectDeserializer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Util.Util;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
/** Add your docs here. */

public class AUTO_Trajectory {
    
    
    String FirstRedBallTrajectoryJSON = "Paths/Unnamed.wpilib.json";
    public Trajectory firstTrajectory;
    public Trajectory secondTrajectory;
    public Trajectory thirdTrajectory;
    public Trajectory fourthTrajectory;
    public Trajectory secondTrajectory2;
    public Trajectory fifthTrajectory;
    // String FirstRedBallTrajectoryJSON = "Paths/FirstRedBall.wpilib.json";
    Trajectory FirstRedBall = new Trajectory();
    private SwerveDrivetrain m_drivetrain;
    public Trajectory FirstRedBallTrajectory;

    public AUTO_Trajectory(SwerveDrivetrain drivetrain){
        m_drivetrain = drivetrain;

        TrajectoryConfig config =
            new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);

            try {
                Path FirstRedBallTrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(FirstRedBallTrajectoryJSON);
                FirstRedBall = TrajectoryUtil.fromPathweaverJson(FirstRedBallTrajectoryPath);
            } catch (IOException e) {
                DriverStation.reportError("Unable to open trajectory", e.getStackTrace());
        }           
      
        FirstRedBallTrajectory = FirstRedBall;
        //Rotation2d uses RADIANS NOT DEGREES!
        //Use Rotation2d.fromDegrees(desiredDegree) instead
      
        firstTrajectory =
        TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(),
        new Pose2d(Units.inchesToMeters(34), 0, new Rotation2d(0)),
        config);

        secondTrajectory =
        TrajectoryGenerator.generateTrajectory(
        new Pose2d(Units.inchesToMeters(40),0, new Rotation2d(0)),
        List.of(new Translation2d(Units.inchesToMeters(-6), Units.inchesToMeters(-0))),
        new Pose2d(Units.inchesToMeters(-24),Units.inchesToMeters(-81), new Rotation2d(Math.toRadians(90))), 
        config);
        
        // secondTrajectory2 =
        // TrajectoryGenerator.generateTrajectory(
        // new Pose2d(Units.inchesToMeters(-1),Units.inchesToMeters(-105), new Rotation2d(Math.toRadians(45))),
        // List.of(),
        // new Pose2d(Units.inchesToMeters(-24),Units.inchesToMeters(-85), new Rotation2d(Math.toRadians(45))), 
        // config);

        thirdTrajectory = 
        TrajectoryGenerator.generateTrajectory(
        new Pose2d(Units.inchesToMeters(-24),Units.inchesToMeters(-81), new Rotation2d(Math.toRadians(45))),    
        List.of(new Translation2d(Units.inchesToMeters(10),Units.inchesToMeters(-150))),
        new Pose2d(Units.inchesToMeters(10),Units.inchesToMeters(-252), new Rotation2d(Math.toRadians(90))), 
        config);

        fourthTrajectory = 
        TrajectoryGenerator.generateTrajectory(
        new Pose2d(Units.inchesToMeters(10),Units.inchesToMeters(-252), new Rotation2d(Math.toRadians(90))),
        List.of(),
        new Pose2d(Units.inchesToMeters(12),Units.inchesToMeters(-81), new Rotation2d(Math.toRadians(90))),
        config);
        
        // fifthTrajectory = 
        // TrajectoryGenerator.generateTrajectory(
        //     new Pose2d(Units.inchesToMeters(12),Units.inchesToMeters(-81), new Rotation2d(Math.toRadians(90))),
        //     List.of(),
        //     new Pose2d(Units.inchesToMeters(12),Units.inchesToMeters(-81), new Rotation2d(Math.toRadians(180))),
        //     config);
    }
  
    public Command driveTrajectory(Trajectory trajectory) {
     
        // Create config for trajectory
        var thetaController =
            new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        // thetaController.setTolerance(1);

        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            m_drivetrain::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_drivetrain::setModuleStates,
            m_drivetrain);


     
        // Reset odometry to the starting pose of the trajectory.
        // m_drivetrain.resetOdometry(trajectory.getInitialPose());
        
        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_drivetrain.drive(0.0 ,0.0 ,0.0, true));
    }


}