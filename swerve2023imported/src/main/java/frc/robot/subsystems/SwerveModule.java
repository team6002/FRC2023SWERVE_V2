// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.DriveConstants;

public class SwerveModule {
  private double m_turningMotorChannel; 
  private final CANSparkMax m_drivingMotor;
  private final CANSparkMax m_turningMotor;
  
  private final DutyCycleEncoder m_turningDutyCycleEncoder;
  
  private final RelativeEncoder m_drivingEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final SparkMaxPIDController m_drivingPIDController;
  private final SparkMaxPIDController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public SwerveModule(
    int drivingCANId, 
    int turningCANId,
    int p_turningDutyCycleEncoder, 
    double chassisAngularOffset, 
    int p_turningMotorChannel) {
    m_drivingMotor = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningCANId, MotorType.kBrushless);
    
    m_turningMotorChannel = p_turningMotorChannel;
    //absoulute encoder stuff
    m_turningDutyCycleEncoder = new DutyCycleEncoder(p_turningDutyCycleEncoder);
    m_turningDutyCycleEncoder.reset();
    m_turningDutyCycleEncoder.setDutyCycleRange(0.05,1);
    m_turningDutyCycleEncoder.setDistancePerRotation(2 *Math.PI);
    
    // m_turningDutyCycleEncoder.setDistancePerRotation(360);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drivingMotor.restoreFactoryDefaults();
    m_turningMotor.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingMotor.getEncoder();
    // m_turningEncoder = m_turningMotor.getAbsoluteEncoder(Type.kDutyCycle);
    m_turningEncoder = m_turningMotor.getEncoder();
    m_drivingPIDController = m_drivingMotor.getPIDController();
    m_turningPIDController = m_turningMotor.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_drivingEncoder.setPositionConversionFactor(DriveConstants.kDriveEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(DriveConstants.kDriveEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(DriveConstants.kTurnEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(DriveConstants.kTurnEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    // m_turningDutyCycleEncoder.setInverted(DriveConstants.kTurnEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    // m_turningPIDController.setPositionPIDWrappingEnabled(true);
    // m_turningPIDController.setPositionPIDWrappingMinInput(DriveConstants.kTurnEncoderPositionPIDMinInput);
    // m_turningPIDController.setPositionPIDWrappingMaxInput(DriveConstants.kTurnEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_drivingPIDController.setP(DriveConstants.kDriveP);
    m_drivingPIDController.setI(DriveConstants.kDriveI);
    m_drivingPIDController.setD(DriveConstants.kDriveD);
    m_drivingPIDController.setFF(DriveConstants.kDriveF);
    // m_drivingPIDController.setOutputRange(DriveConstants.kDriveMinOutput,
        // DriveConstants.kDriveMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(DriveConstants.kTurnP);
    m_turningPIDController.setI(DriveConstants.kTurnI);
    m_turningPIDController.setD(DriveConstants.kTurnD);
    m_turningPIDController.setFF(DriveConstants.kTurnFF);
    // m_turningPIDController.setOutputRange(DriveConstants.kTurnMinOutput,
        // DriveConstants.kTurnMaxOutput);

    m_drivingMotor.setIdleMode(DriveConstants.kDriveMotorIdleMode);
    m_turningMotor.setIdleMode(DriveConstants.kTurnMotorIdleMode);
    m_drivingMotor.setSmartCurrentLimit(DriveConstants.kDriveMotorCurrentLimit);
    m_turningMotor.setSmartCurrentLimit(DriveConstants.kTurnMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingMotor.burnFlash();
    m_turningMotor.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);

    syncAngle();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  // public SwerveModuleState getState() {
  //   // Apply chassis angular offset to the encoder position to get the position
  //   // relative to the chassis.
  //   return new SwerveModuleState(m_drivingEncoder.getVelocity(),
  //       new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  // }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(getAngle() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_turningEncoder.setPosition(0);
  }

  // gets te absolute angle
  public double getAbsoluteAngle() {
    // set the lamprey sensor to zero degree pointing to the back
    // the analog signal is zero to 3.3v, representing 0 to 360 degree, CCW positive
    
    // lamprey attached to sparmkmax
    // double absoluteAngle = m_analogSensor.getPosition();
  
    // lamprey attached to roborio
    // double absoluteAngle = 2*Math.PI - m_analogRoborioPort.get();
    double absoluteAngle =  2*Math.PI - (2*Math.PI * m_turningDutyCycleEncoder.getAbsolutePosition());
    
    return absoluteAngle;
  }
  // gets the relative angle
  public double getAngle() {
    // Note: This assumes the CANCoders are setup with the default feedback coefficient
    // and the sensor value reports degrees.
    // double angle = ((m_turningEncoder.getPosition()) % 2*Math.PI);// - m_turningEncoderOffset
    // double angle = (m_analogSensor.getPosition() - m_turningEncoderOffset) % 360;
    // if (angle > Math.PI) angle-=2*Math.PI;
    
    return m_turningEncoder.getPosition();
  }

  public void resetDriveEnc(){
    m_drivingEncoder.setPosition(0);
  }

  public void syncAngle() {
    m_turningEncoder.setPosition(getAbsoluteAngle());
  }

  public SwerveModuleState getState() {
    // double fakeSpeed = -0.16666;
    return new SwerveModuleState(m_drivingEncoder.getVelocity(), new Rotation2d(getAngle())); //* (Math.PI / 180.0)));
    // return new SwerveModuleState(fakeSpeed, new Rotation2d(Math.PI / 2));
  }
  

  public void updateSmartDashboard() {
    //       double angle = getAbsAngle();
    //       angle -= m_turningEncoderOffset;
    //       if (angle > 180) angle -= 360;
  
          // // SmartDashboard.putNumber("turnRaw:"+m_turningMotorChannel, m_turningEncoder.getPosition());
          SmartDashboard.putNumber("TurnRelativeEncoder:"+m_turningMotorChannel, Math.toDegrees(m_turningEncoder.getPosition())  );
          SmartDashboard.putNumber("TurnRawAbsoluteEncoder"+m_turningMotorChannel,(m_turningDutyCycleEncoder.getAbsolutePosition()) );
          SmartDashboard.putNumber("TurnAbsoluteEncoder"+m_turningMotorChannel,(Math.toDegrees(getAbsoluteAngle())));
          // SmartDashboard.putBoolean("Connected"+m_turningMotorChannel,(m_turningDutyCycleEncoder.isConnected()));
          // SmartDashboard.putNumber("Frequency"+m_turningMotorChannel,(m_turningDutyCycleEncoder.getFrequency()));
          // SmartDashboard.putNumber("position"+m_turningMotorChannel,(m_turningDutyCycleEncoder.get()));
          // SmartDashboard.putNumber("turning"+m_turningMotorChannel,(m_turningDutyCycleEncoder.getDistance()));
          

    }
  
}