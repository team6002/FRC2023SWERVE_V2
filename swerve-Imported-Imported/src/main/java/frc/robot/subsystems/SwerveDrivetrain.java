// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;



public class SwerveDrivetrain extends SubsystemBase {

  // public static final double kMaxSpeed = Units.feetToMeters(10); // 13.6 feet per second/ 10 feet per second now
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  

  /**
   * TODO: These are example values and will need to be adjusted for your robot!
   * Modules are in the order of -
   * Front Left
   * Front Right
   * Back Left
   * Back Right
   * 
   * Positive x values represent moving toward the front of the robot whereas
   * positive y values represent moving toward the left of the robot
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#constructing-the-kinematics-object
   */


  private final SwerveModule m_frontLeft = 
    new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort
        , DriveConstants.kFrontLeftTurningMotorPort
        , DriveConstants.kFrontLeftDriveMotorInverted
        , DriveConstants.kFrontLeftDriveAnalogPort);
  private final SwerveModule m_frontRight = 
    new SwerveModule(DriveConstants.kFrontRightDriveMotorPort
        , DriveConstants.kFrontRightTurningMotorPort
        , DriveConstants.kFrontRightDriveMotorInverted
        , DriveConstants.kFrontRightDriveAnalogPort);
  private final SwerveModule m_rearLeft = 
    new SwerveModule(DriveConstants.kRearLeftDriveMotorPort
        , DriveConstants.kRearLeftTurningMotorPort
        , DriveConstants.kRearLeftDriveMotorInverted
        , DriveConstants.kRearLeftDriveAnalogPort);
  private final SwerveModule m_rearRight = 
    new SwerveModule(DriveConstants.kRearRightDriveMotorPort
        , DriveConstants.kRearRightTurningMotorPort
        , DriveConstants.kRearRightDriveMotorInverted
        , DriveConstants.kRearRightDriveAnalogPort);
  
  private final AHRS m_Navx = new AHRS(Port.kMXP);
  private boolean fieldMode = false;

  public final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_Navx.getRotation2d());//, new Pose2d(0, 0, new Rotation2d()

  public SwerveDrivetrain() {
   
    syncAllAngles();
  
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_Navx.zeroYaw();
  }

  public Rotation2d getheading() {
    if (m_Navx.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(m_Navx.getFusedHeading());
    }
 
    //counter-clockwise = positive angle
    return Rotation2d.fromDegrees(-m_Navx.getYaw());
  }

  // public void setHeading(double rotation){
  //   m_Navx.zeroYaw();
  //   m_Navx.setAngleAdjustment(rotation);
  // }

  public void syncAllAngles() {
    m_frontLeft.syncAngle();
    m_frontRight.syncAngle();
    m_rearLeft.syncAngle();
    m_rearRight.syncAngle();
  }
  
  public void resetAllAngles(){
    m_frontLeft.resetDriveEnc();
    m_frontRight.resetDriveEnc();
    m_rearLeft.resetDriveEnc();
    m_rearRight.resetDriveEnc();
  }

       
  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_Navx.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    setModuleStates(swerveModuleStates);
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
  
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public double getOdometryX(){
    return m_odometry.getPoseMeters().getX();
  }
  
  public double getOdometryY(){
    return m_odometry.getPoseMeters().getY();
  }

  public double getOdometryRotate(){
    return m_odometry.getPoseMeters().getRotation().getDegrees();
  }
  
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_Navx.getRotation2d());
  }

  public double getDegrees(){
    return m_Navx.getRotation2d().getDegrees();
  }

 
  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    // var gyroAngle = Rotation2d.fromDegrees(-m_Navx.getYaw());
    m_odometry.update(
        // gyroAngle,
        m_Navx.getRotation2d(),
        // new Rotation2d(0),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  public void resetDriveEncoder(){
    m_frontLeft.resetDriveEnc();
    m_frontRight.resetDriveEnc();
    m_rearLeft.resetDriveEnc();
    m_rearRight.resetDriveEnc();
  }

  public void fieldModeChange(){
    fieldMode = !fieldMode;
  }

  public boolean getFieldMode(){
    return fieldMode;
  }
  
  public void updateSmartDashboard() {
    // SmartDashboard.putBoolean("FieldRelative", fieldMode);
    SmartDashboard.putNumber("OdoX", getOdometryX());
    SmartDashboard.putNumber("OdoY", getOdometryY());
    SmartDashboard.putNumber("OdoRotate", getOdometryRotate());
    m_frontLeft.updateSmartDashboard();
    m_frontRight.updateSmartDashboard();
    m_rearLeft.updateSmartDashboard();
    m_rearRight.updateSmartDashboard();
    
    SmartDashboard.putNumber("NavxDegrees", m_Navx.getRotation2d().getDegrees());
    SmartDashboard.putNumber("NavxRadians", m_Navx.getRotation2d().getRadians());

  };


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
    updateSmartDashboard();
  }

  @Override       
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}