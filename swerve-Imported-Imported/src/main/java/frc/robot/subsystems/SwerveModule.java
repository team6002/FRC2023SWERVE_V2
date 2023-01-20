// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANSparkMaxLowLevel;
// import com.revrobotics.AnalogInput;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {

  private static final double kDriveP = 0.005;  //15.0;
  private static final double kDriveI = 0;//0;  //0.01;
  private static final double kDriveD = 0.1;  //0.1;
  private static final double kDriveF = 0.2;  //0.2;

  private static final double kAngleP = 1.0;
  private static final double kAngleI = 0.00;
  private static final double kAngleD = 0.00;
  private static final double kAngleFF = 0.00;
  // private static final double kModuleMaxAngularVelocity = SwerveDrivetrain.kMaxAngularSpeed;
  // private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;
  private final double m_turningMotorChannel;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final SparkMaxPIDController m_drivePIDController;
  private final SparkMaxPIDController m_turningPIDController;
 
  private final SparkMaxAnalogSensor m_analogSensor;
  private final int m_analogPortID;
  private final AnalogPotentiometer m_analogRoborioPort;
  
  private static final int ENCODER_RESET_ITERATIONS = 500;
  private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);
  private double referenceAngleRadians = 0;
  private double resetIteration = 0;


  public SwerveModule(
    int driveMotorChannel,
    int turningMotorChannel,
    boolean driveDirection
    , int p_analogPortID) {

    m_turningMotorChannel = turningMotorChannel;    
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
    m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
    m_driveMotor.setInverted(driveDirection);
    
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
    m_turningMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    m_turningMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
    
    /**
   * The restoreFactoryDefaults method can be used to reset the configuration parameters
   * in the SPARK MAX to their factory default state. If no argument is passed, these
   * parameters will not persist between power cycles
   */
 
    // m_driveMotor.restoreFactoryDefaults();
    // m_turningMotor.restoreFactoryDefaults();

    m_driveEncoder = m_driveMotor.getEncoder();

    double kwheeldiameter = Units.inchesToMeters(3);   // 3in 
    double kdriveReduction = (16.0 / 32.0) * (15.0 / 45.0);
    double positionConversionFactor = Math.PI * kwheeldiameter * kdriveReduction;
    m_driveEncoder.setPositionConversionFactor(positionConversionFactor);
    m_driveEncoder.setVelocityConversionFactor(positionConversionFactor / 60.0);
    m_driveEncoder.setPosition(0);
  
    m_turningEncoder = m_turningMotor.getEncoder();
    double ksteeringReduction = (10.0 / 30.0 ) * (18.0 / 96.0);
    m_turningEncoder.setPositionConversionFactor(2 * Math.PI * ksteeringReduction);
    m_turningEncoder.setVelocityConversionFactor(2 * Math.PI * ksteeringReduction / 60);
      
    // Lamprey encoder attached to the sparkmax
    m_analogSensor = m_turningMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
    m_analogSensor.setInverted(true);
    double analogPositionConversionFactor = (2 * Math.PI / 3.3); // analog signal => 0 to 3.3
    m_analogSensor.setPositionConversionFactor(analogPositionConversionFactor);


    // Lamprey encoder attached to the roborio
    m_analogPortID = p_analogPortID;
    m_analogRoborioPort =  new AnalogPotentiometer(m_analogPortID, 2 * Math.PI * (5 / 3.3), 0);



    m_turningPIDController = m_turningMotor.getPIDController();
    // m_turningPIDController.enableContinuousInput(-Math.PI,Math.PI);
    // m_turningPIDController.setFeedbackDevice(m_analogSensor);
    m_turningPIDController.setP(kAngleP);
    m_turningPIDController.setI(kAngleI);
    m_turningPIDController.setD(kAngleD);
    m_turningPIDController.setFF(kAngleFF);

    m_drivePIDController = m_driveMotor.getPIDController();
    m_drivePIDController.setFeedbackDevice(m_driveEncoder);
    m_drivePIDController.setP(kDriveP);
    m_drivePIDController.setI(kDriveI);
    m_drivePIDController.setD(kDriveD);
    m_drivePIDController.setFF(kDriveF);

    syncAngle();

}

public double getAbsoluteAngle() {
  // set the lamprey sensor to zero degree pointing to the back
  // the analog signal is zero to 3.3v, representing 0 to 360 degree, CCW positive
  
  // lamprey attached to sparmkmax
  // double absoluteAngle = m_analogSensor.getPosition();

  // lamprey attached to roborio
  double absoluteAngle = 2*Math.PI - m_analogRoborioPort.get();

  return absoluteAngle;
}



    /**
   * Gets the relative rotational position of the module
   * @return The relative rotational position of the angle motor in degrees
   */
  public double getAngle() {
    // Note: This assumes the CANCoders are setup with the default feedback coefficient
    // and the sensor value reports degrees.
    // double angle = ((m_turningEncoder.getPosition()) % 2*Math.PI);// - m_turningEncoderOffset
    // double angle = (m_analogSensor.getPosition() - m_turningEncoderOffset) % 360;
    // if (angle > Math.PI) angle-=2*Math.PI;
    
    return m_turningEncoder.getPosition();
  }
 
  public void updateSmartDashboard() {
  //       double angle = getAbsAngle();
  //       angle -= m_turningEncoderOffset;
  //       if (angle > 180) angle -= 360;

        // // SmartDashboard.putNumber("turnRaw:"+m_turningMotorChannel, m_turningEncoder.getPosition());
        SmartDashboard.putNumber("TurnRelativeEncoder:"+m_turningMotorChannel, Math.toDegrees(m_turningEncoder.getPosition())  );
        SmartDashboard.putNumber("TurnAbsoluteEncoder"+m_turningMotorChannel,Math.toDegrees(getAbsoluteAngle()) );
        SmartDashboard.putNumber("Desired Rotation"+m_turningMotorChannel, resetIteration);
        // SmartDashboard.putNumber("resetIteration"+m_turningMotorChannel, resetIteration);
        // SmartDashboard.putNumber("driveEnc:"+m_turningMotorChannel, m_driveEncoder.getPosition());
        // SmartDashboard.putNumber("rawAnalogEnc:"+m_turningMotorChannel, m_analogSensor.getPosition());
        // SmartDashboard.putNumber("turnAbs:"+m_turningMotorChannel, angle);
        // SmartDashboard.putNumber("driveVelocity"+m_turningMotorChannel, m_driveEncoder.getVelocity());
        // SmartDashboard.putNumber("turnOff:"+m_turningMotorChannel, m_turningEncoderOffset);
  }

  /**
   * Resets the relative encoder to the absolute encoder.
   */
  
   public void syncAngle() {
    m_turningEncoder.setPosition(getAbsoluteAngle());
  }


  public void resetDriveEnc(){
    m_driveEncoder.setPosition(0);
  }


  // public void setAngle(Rotation2d rotation) {
  //   double angle = rotation.getDegrees();
  //   angle+=180;
  //   m_turningPIDController.setReference(angle, ControlType.kPosition);
    
  // }
 
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // double fakeSpeed = -0.16666;
    return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(getAngle())); //* (Math.PI / 180.0)));
    // return new SwerveModuleState(fakeSpeed, new Rotation2d(Math.PI / 2));
  }


  public void setReferenceAngle(double referenceAngleRadians)  {

    double currentAngleRadians = m_turningEncoder.getPosition();

    // Reset the NEO's encoder periodically when the module is not rotating.
    // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
    // end up getting a good reading. If we reset periodically this won't matter anymore.
    if (m_turningEncoder.getVelocity() < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
      if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
        resetIteration = 0;
        double absoluteAngle = getAbsoluteAngle();
        m_turningEncoder.setPosition(absoluteAngle);
        currentAngleRadians = absoluteAngle;
      }
    } else {
      resetIteration = 0;
    }

    double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
    if (currentAngleRadiansMod < 0.0) {
      currentAngleRadiansMod += 2.0 * Math.PI;
    }

    // The reference angle has the range [0, 2pi) but the Neo's encoder can go above that
    double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
    if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
      adjustedReferenceAngleRadians -= 2.0 * Math.PI;
    } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
      adjustedReferenceAngleRadians += 2.0 * Math.PI;
    }

    this.referenceAngleRadians = referenceAngleRadians;
    
    SmartDashboard.putNumber("Desired Rotation"+m_turningMotorChannel, Math.toDegrees(adjustedReferenceAngleRadians));
        
    m_turningPIDController.setReference(adjustedReferenceAngleRadians, ControlType.kPosition);

  }


  /**
   * 
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    Rotation2d currentRotation = Rotation2d.fromDegrees(getAngle());
     // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, currentRotation);

      setReferenceAngle(state.angle.getRadians());
    // setAngle(state.angle);

    // SmartDashboard.putNumber("TurnC:"+m_turningMotorChannel, currentRotation.getDegrees());
    // SmartDashboard.putNumber("TurnD:"+m_turningMotorChannel, state.angle.getDegrees());


    // double feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond);
    double speed = state.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond;
    m_driveMotor.set(speed);
        
  }
  
  public void stop() {
    m_driveMotor.set(0);
    m_turningMotor.set(0);
  } 
}
 
 