// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants.*;
import frc.robot.subsystems.SUB_FiniteStateMachine.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Drivetrain extends SubsystemBase {
  // Create SwerveModules
  SUB_Blinkin m_blinkin;
  SUB_LimeLight m_LimeLight;
  SUB_FiniteStateMachine m_finiteStateMachine;
  public boolean balanced = false;
  public double timer;
  public int wantedHeight;//wanted position for droppoff, integer from 1-3, 1 is bottom, 3 is top
  public int wantedLength;//wanted position for droppoff, integer from 1-3, 1 is far left, 3 is far right
  public double[] relativeBotpose;

  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveID,
      DriveConstants.kFrontLeftTurningID,
      DriveConstants.kFrontLeftOffset,
      "FrontLeft"
      );
  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveID,
      DriveConstants.kFrontRightTurningID,
      DriveConstants.kFrontRightOffset,
      "FrontRight"
      );

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kBackLeftDriveID,
      DriveConstants.kBackLeftTurningID,
      DriveConstants.kBackLeftOffset,
      "BackLeft"
      );

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kBackRightDriveID,
      DriveConstants.kBackRightTurningID,
      DriveConstants.kBackRightOffset,
      "BackRight"
      );

  // The gyro sensor
  private final AHRS m_navx = new AHRS(Port.kMXP);
  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_navx.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new SUB_Drivetrain. */
  public SUB_Drivetrain(SUB_Blinkin p_blinkin, SUB_FiniteStateMachine p_finiteStateMachine, SUB_LimeLight p_limeLight) {
    m_blinkin = p_blinkin;
    m_finiteStateMachine = p_finiteStateMachine;
    m_LimeLight = p_limeLight;
  }

  public void updateDashboardDrive(){
    m_frontRight.updateSmartDashboard();
    m_frontLeft.updateSmartDashboard();
    m_rearLeft.updateSmartDashboard();
    m_rearRight.updateSmartDashboard();

    SmartDashboard.putNumber("m_navx.getAngle()", m_navx.getAngle());
    SmartDashboard.putNumber("m_navx.getRotation2d()", m_navx.getRotation2d().getDegrees());
  }

  @Override
  public void periodic() {
    updateDashboardDrive();
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_navx.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
        
        if(m_finiteStateMachine.getState() == RobotState.BALANCING){
          if(Math.abs(getPitch()) < 5){
              m_blinkin.set(BlinkinConstants.kGreen);
              balanced = true;
              //drive stop
          }else if(getPitch() > 30){
            m_blinkin.setBackFar();
            balanced = false;
            //drive forward
          }else if(getPitch() > 15){
            m_blinkin.setBack();
            balanced = false;
            //drive forward slowly
          }else if(getPitch() < -30){
            m_blinkin.setFrontFar();
            balanced = false;
            //drive back slowly
          }else if(getPitch() < -15){
            m_blinkin.setFront();
            balanced = false;
            //drive back
          }
          
          if(balanced == true) timer +=.02;
          else timer = 0;

          if(timer >= 2){
            m_blinkin.setCelebrate();
            timer = 2;
        }
      }
     relativeBotpose = m_LimeLight.getRelativePosition(m_LimeLight.getTargetID());


        telemetry();
  }

  public void telemetry(){
    SmartDashboard.putNumber("pitch", getPitch());
    SmartDashboard.putNumber("timer", timer);
    SmartDashboard.putNumber("x", Units.metersToInches(relativeBotpose[0]));
    SmartDashboard.putNumber("y", Units.metersToInches(relativeBotpose[1]));
    SmartDashboard.putNumber("yaw", Units.metersToInches(relativeBotpose[2]));
    SmartDashboard.putNumber("raw x", getX());
    SmartDashboard.putNumber("raw y", getY());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public double getX(){
    return m_odometry.getPoseMeters().getX();
  }

  public double getY(){
    return m_odometry.getPoseMeters().getY();
  }

  public double getPitch(){
    return m_navx.getPitch();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_navx.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Adjust input based on max speed
    xSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    ySpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    rot *= DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_navx.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_navx.zeroYaw();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_navx.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_navx.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void setWantedHeight(int p_height){
    wantedHeight = p_height;
  }

  public void setWantedLength(int p_length){
    wantedLength = p_length;
  }

  public int getWantedLength(){
    return wantedLength;
  }
}