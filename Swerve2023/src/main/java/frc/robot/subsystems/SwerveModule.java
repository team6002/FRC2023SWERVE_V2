// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.DriveConstants;

public class SwerveModule {
  private final CANSparkMax m_drivingSparkMax;
  private final CANSparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkMaxPIDController m_drivingPIDController;
  private final SparkMaxPIDController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  private final String m_moduleChannelStr;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, String channel_string) {
    m_moduleChannelStr = channel_string;

    m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drivingSparkMax.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_drivingPIDController = m_drivingSparkMax.getPIDController();
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_drivingEncoder.setPositionConversionFactor(DriveConstants.kDriveEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(DriveConstants.kDriveEncoderVelocityFactor);

    System.out.println("Vel Conversion Factor: " + DriveConstants.kDriveEncoderVelocityFactor);
    System.out.println("Pos Conversion Factor: " + DriveConstants.kDriveEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(DriveConstants.kTurnEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(DriveConstants.kTurnEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    m_turningEncoder.setInverted(DriveConstants.kTurnEncoderInverted);
    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(DriveConstants.kTurnEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(DriveConstants.kTurnEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_drivingPIDController.setP(DriveConstants.kDriveP);
    m_drivingPIDController.setI(DriveConstants.kDriveI);
    m_drivingPIDController.setD(DriveConstants.kDriveD);
    m_drivingPIDController.setFF(DriveConstants.kDriveF);
    m_drivingPIDController.setOutputRange(DriveConstants.kDriveMinOutput,
        DriveConstants.kDriveMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(DriveConstants.kTurnP);
    m_turningPIDController.setI(DriveConstants.kTurnI);
    m_turningPIDController.setD(DriveConstants.kTurnD);
    m_turningPIDController.setFF(DriveConstants.kTurnF);
    m_turningPIDController.setOutputRange(DriveConstants.kTurnMinOutput,
        DriveConstants.kTurnMaxOutput);

    m_drivingSparkMax.setIdleMode(DriveConstants.kDriveMotorIdleMode);
    m_turningSparkMax.setIdleMode(DriveConstants.kTurnMotorIdleMode);
    m_drivingSparkMax.setSmartCurrentLimit(DriveConstants.kDriveMotorCurrentLimit);
    m_turningSparkMax.setSmartCurrentLimit(DriveConstants.kTurnMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingSparkMax.burnFlash();
    m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

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
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
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
    // m_optimizedDesiredState = optimizedDesiredState.angle.getRadians();
    m_desiredState = desiredState;
    // m_optimizedDesiredSpeed = optimizedDesiredState.speedMetersPerSecond;
  }
//   private double m_optimizedDesiredState = 0;
//   private double m_optimizedDesiredSpeed = 0;
  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
  public void updateSmartDashboard(){
    // // SmartDashboard.putNumber("speed", m_turningSparkMax.get());
    // SmartDashboard.putNumber(m_moduleChannelStr+"DesiredAngle", m_optimizedDesiredState);
    SmartDashboard.putNumber(m_moduleChannelStr+"AbsoluteAngle", m_turningEncoder.getPosition());
    // SmartDashboard.putNumber(m_moduleChannelStr+"AbsoluteAngleinDegrees", Math.toDegrees(m_turningEncoder.getPosition()));
    // SmartDashboard.putNumber(m_moduleChannelStr+"DesiredSpeed", m_optimizedDesiredSpeed);
    // SmartDashboard.putNumber(m_moduleChannelStr+"MeasuredSpeed", m_drivingEncoder.getVelocity());
    // SmartDashboard.putNumber(m_moduleChannelStr+"DistanceTravled", m_drivingEncoder.getPosition());
    }
}