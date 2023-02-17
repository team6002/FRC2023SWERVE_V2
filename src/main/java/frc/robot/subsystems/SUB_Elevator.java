// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Elevator extends SubsystemBase {
  private final CANSparkMax m_elevatorMotor;
  private final SparkMaxPIDController m_elevatorMotorPIDController;
  private final RelativeEncoder m_elevatorEncoder;

    public SUB_Elevator() {
      m_elevatorMotor = new CANSparkMax(ElevatorConstants.kElevatorMotorCanID, MotorType.kBrushless);
      m_elevatorMotorPIDController = m_elevatorMotor.getPIDController();
      m_elevatorEncoder = m_elevatorMotor.getEncoder();
      m_elevatorMotor.restoreFactoryDefaults();
      m_elevatorEncoder.setPositionConversionFactor(.641);
      m_elevatorEncoder.setVelocityConversionFactor(0.641/60);
      m_elevatorMotor.setInverted(true);
      m_elevatorMotorPIDController.setP(ElevatorConstants.kElevatorP,1);
      m_elevatorMotorPIDController.setI(ElevatorConstants.kElevatorI,1);
      m_elevatorMotorPIDController.setD(ElevatorConstants.kElevatorD,1);
      m_elevatorMotorPIDController.setFF(ElevatorConstants.kElevatorF,1);
      m_elevatorMotorPIDController.setFeedbackDevice(m_elevatorEncoder);
      m_elevatorMotor.setIdleMode(IdleMode.kCoast);
      m_elevatorMotorPIDController.setPositionPIDWrappingEnabled(false);
      m_elevatorMotorPIDController.setOutputRange(-1, 1, 1);
      m_elevatorMotorPIDController.setSmartMotionMaxVelocity(40, 1);
      m_elevatorMotorPIDController.setSmartMotionMaxAccel(40, 1);
      m_elevatorMotorPIDController.setSmartMotionAllowedClosedLoopError(1, 1);
      m_elevatorMotorPIDController.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 1);
      m_elevatorMotor.burnFlash();
      m_elevatorEncoder.setPosition(0);
    }

    public void setPosition(double p_reference){
      m_elevatorMotorPIDController.setReference(p_reference, CANSparkMax.ControlType.kSmartMotion,1);
    } 
    public double getPosition(){
      return m_elevatorEncoder.getPosition();
    }
  public void setElevatorOn(){
    // m_elevatorMotor.set(ElevatorConstants.kElevatorForward);
    m_elevatorMotor.set(.2);
  }

  public void setElevatorOff(){
    m_elevatorMotorPIDController.setReference(m_elevatorEncoder.getPosition(), ControlType.kSmartMotion);
  }

  public void setElevatorReverse(){
    // m_elevatorMotor.set(-ElevatorConstants.kElevatorForward);
    m_elevatorMotor.set(-.2);
  }

  @Override
  public void periodic() {
    // updates elevator telemetry
    telemetry();
  }

  public void setPower(double p_power){
    m_elevatorMotor.set(p_power);
  }
  
  // double m_P = 0;//elevatorConstants.kelevatorP;
  // double m_I = 0;//elevatorConstants.kelevatorI;
  // double m_D = 0;//elevatorConstants.kelevatorD;
  // double m_F = 0;//elevatorConstants.kelevatorF;
  double m_wantedPosition = 0;
  public void telemetry(){
    SmartDashboard.putNumber("elevator Position", m_elevatorEncoder.getPosition());
    SmartDashboard.putNumber("elevator Position (numeric)", m_elevatorEncoder.getPosition());

    // m_P = SmartDashboard.getNumber("P", m_P);
    // m_I = SmartDashboard.getNumber("I", m_I);
    // m_D = SmartDashboard.getNumber("D", m_D);
    // m_F = SmartDashboard.getNumber("F", m_F);
    // m_wantedPosition = SmartDashboard.getNumber("wantedPosition", m_wantedPosition);

    // SmartDashboard.putNumber("P", m_P);
    // SmartDashboard.putNumber("I", m_I);
    // SmartDashboard.putNumber("D", m_D);
    // SmartDashboard.putNumber("F", m_F);
    // SmartDashboard.putNumber("wantedPosition", m_wantedPosition);
   
    // m_elevatorMotorPIDController.setP(m_P,1);
    // m_elevatorMotorPIDController.setI(m_I,1);
    // m_elevatorMotorPIDController.setD(m_D,1);
    // m_elevatorMotorPIDController.setFF(m_F,1);
    // m_elevatorMotorPIDController.setReference(m_wantedPosition, ControlType.kSmartMotion, 1);

    // SmartDashboard.putNumber("velocity", m_elevatorEncoder.getVelocity());
    // SmartDashboard.putNumber("output", m_elevatorMotor.getAppliedOutput());
    // SmartDashboard.putNumber("wantedspeed", m_elevatorMotor.get());
   }
}
