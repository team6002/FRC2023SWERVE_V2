// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autos.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  // private Timer tim = new Timer();
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  SendableChooser<Command> auto = new SendableChooser<Command>();
  

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // m_robotContainer.m_NavxGyro.resetNavx();
    m_robotContainer.m_drivetrain.syncAllAngles();
    m_robotContainer.m_drivetrain.zeroHeading();
    m_robotContainer.m_drivetrain.resetDriveEncoder();
    m_robotContainer.m_drivetrain.resetOdometry(new Pose2d(0,0, new Rotation2d(0)));
    LiveWindow.disableAllTelemetry();
    // auto.addOption("Rightside ThreeBall", 
    //                 new AUTO_ThreeBall(m_robotContainer.m_turret, m_robotContainer.m_intake, 
    //                                   m_robotContainer.m_intakeStatus, m_robotContainer.m_shooter, 
    //                                   m_robotContainer.m_drivetrain, m_robotContainer.m_autotrajectory));
    // SmartDashboard.putData("Auto Mode", auto);
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible ands, removing finished or interrupted commands,
    // and running subsystem periodic() methodsfor polling buttons, adding newly-scheduled
    // commands, running already-scheduled comm.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
    // m_robotContainer.updateOdometry();

    // double shooterSetpoint = 
      // SmartDashboard.getNumber("Desired Shooter Setpoint", Constants.ShooterConstants.kShootingVelocity);
    // m_robotContainer.m_shooter.setShooterSetpoint(shooterSetpoint);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.m_drivetrain.syncAllAngles();
    m_robotContainer.m_drivetrain.zeroHeading();
    m_robotContainer.m_drivetrain.resetDriveEncoder();
    m_robotContainer.m_drivetrain.resetOdometry(new Pose2d(0,0, new Rotation2d(0)));
    //PLACE AUTONOMOUS COMMANDS HERE
    // mAutonomousCommand =
    // new AUTO_TwoBall_Experimental(m_robotContainer.m_turret, m_robotContainer.m_intake, m_robotContainer.m_intakeStatus, 
    //                 m_robotContainer.m_shooter, m_robotContainer.m_drivetrain, m_robotContainer.m_autotrajectory);
    //auto.getSelected();

    m_autonomousCommand = new AUTO_4balls(m_robotContainer.m_drivetrain, m_robotContainer.m_intake, 
                                    m_robotContainer.m_intakeStatus,m_robotContainer.m_trajectory, m_robotContainer.m_shooter);
    
    

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // m_robotContainer.m_drivetrain.resetDriveEncoder();
    // m_robotContainer.m_drivetrain.zeroHeading();
    m_robotContainer.m_drivetrain.syncAllAngles();
    // m_robotContainer.m_drivetrain.resetOdometry(new Pose2d(0,0, new Rotation2d(0)));
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}


}
