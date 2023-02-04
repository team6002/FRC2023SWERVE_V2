// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Arm extends SubsystemBase {
  private final CANSparkMax m_armMotor;
  private final SparkMaxPIDController m_armMotorPIDController;
  private final AbsoluteEncoder m_armEncoder;
  private double m_wantedPosition;
  double m_arm_down_p, m_arm_down_i, m_arm_down_d, m_arm_down_f;
  private int m_wantedSlot;

    public SUB_Arm() {
      m_armMotor = new CANSparkMax(ArmConstants.kArmMotorCanID, MotorType.kBrushless);
      m_armMotorPIDController = m_armMotor.getPIDController();
      m_armEncoder = m_armMotor.getAbsoluteEncoder(Type.kDutyCycle);
      m_armEncoder.setPositionConversionFactor(360);
      m_armEncoder.setVelocityConversionFactor(6);
      m_armEncoder.setInverted(true);
      // m_armMotorPIDController.setP(ArmConstants.kArmP);
      // m_armMotorPIDController.setI(ArmConstants.kArmI);
      // m_armMotorPIDController.setD(ArmConstants.kArmD);
      // m_armMotorPIDController.setFF(ArmConstants.kArmF);
      m_armMotorPIDController.setP(0.00355,1);
      m_armMotorPIDController.setI(0,1);
      m_armMotorPIDController.setD(0,1);
      m_armMotorPIDController.setFF(0.005,1);
      m_armMotorPIDController.setFeedbackDevice(m_armEncoder);
      m_armMotor.setIdleMode(IdleMode.kCoast);
      m_armMotorPIDController.setPositionPIDWrappingEnabled(false);
      m_armMotorPIDController.setOutputRange(-1, 1, 1);
      m_armMotorPIDController.setSmartMotionMaxVelocity(20, 1);
      m_armMotorPIDController.setSmartMotionMaxAccel(10, 1);
      m_armMotorPIDController.setSmartMotionMinOutputVelocity(0, 1);
      m_armMotorPIDController.setSmartMotionAllowedClosedLoopError(1, 1);
      m_armMotorPIDController.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 1);

      m_armMotorPIDController.setP(0.004,2);
      m_armMotorPIDController.setI(0,2);
      m_armMotorPIDController.setD(0,2);
      m_armMotorPIDController.setFF(0.00004,2);
      m_armMotorPIDController.setOutputRange(-1, 1, 2);
      m_armMotorPIDController.setSmartMotionMaxVelocity(10, 2);
      m_armMotorPIDController.setSmartMotionMinOutputVelocity(0, 2);
      m_armMotorPIDController.setSmartMotionMaxAccel(10, 2);
      m_armMotorPIDController.setSmartMotionAllowedClosedLoopError(1, 2);
      m_armMotorPIDController.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 2);

      m_wantedSlot = 1;
    }

    // public void setPosition(double position){
    //   m_armMotorPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    // }
    public void setPosition(double p_reference){
      m_wantedPosition = p_reference;
      if(p_reference > 125){
        m_armMotorPIDController.setP(0.00355,1);
        m_armMotorPIDController.setI(0,1);
        m_armMotorPIDController.setD(0,1);
        m_armMotorPIDController.setFF(0.00025,1);
        m_armMotorPIDController.setSmartMotionMaxVelocity(20, 1);
        m_armMotorPIDController.setSmartMotionMaxAccel(20, 1);
        m_wantedSlot = 1;
        m_armMotorPIDController.setReference(p_reference, CANSparkMax.ControlType.kPosition,1);
      }else{
        m_wantedSlot = 2;
        m_armMotorPIDController.setP(0.004,2);
        m_armMotorPIDController.setI(0,2);
        m_armMotorPIDController.setD(m_arm_down_d,2);
        m_armMotorPIDController.setFF(0.0000,2);
        m_armMotorPIDController.setSmartMotionMaxVelocity(10, 2);
        m_armMotorPIDController.setSmartMotionMaxAccel(2.5, 2);
        m_armMotorPIDController.setReference(p_reference, CANSparkMax.ControlType.kPosition, 2);
      }
    } 
    public double getPosition(){
      return m_armEncoder.getPosition();
    }
  public void setArmOn(){
    m_armMotor.set(ArmConstants.kArmForward);
  }

  public void setArmOff(){
    m_armMotorPIDController.setReference(m_armEncoder.getPosition(), ControlType.kSmartMotion);
  }

  public void setArmReverse(){
    m_armMotor.set(-ArmConstants.kArmForward);
  }

  @Override
  public void periodic() {
    // updates arm telemetry
    telemetry();
  }

  public void telemetry(){

    m_arm_down_p = SmartDashboard.getNumber("Arm DOWN P=", 0);
    m_arm_down_i = SmartDashboard.getNumber("Arm DOWN I=", 0);
    m_arm_down_d = SmartDashboard.getNumber("Arm DOWN D=", 0);
    m_arm_down_f = SmartDashboard.getNumber("Arm DOWN F=", 0);

    SmartDashboard.putNumber("Arm DOWN P=", m_arm_down_p);
    SmartDashboard.putNumber("Arm DOWN I=", m_arm_down_i);
    SmartDashboard.putNumber("Arm DOWN D=", m_arm_down_d);
    SmartDashboard.putNumber("Arm DOWN F=", m_arm_down_f);
    SmartDashboard.putNumber("Arm Position", m_armEncoder.getPosition());
    SmartDashboard.putNumber("Arm Position (numeric)", m_armEncoder.getPosition());

    SmartDashboard.putNumber("velocity", m_armEncoder.getVelocity());
    SmartDashboard.putNumber("output", m_armMotor.getAppliedOutput());
    SmartDashboard.putNumber("wanted speed", m_armMotor.get());
    SmartDashboard.putNumber("ArmSetpoint", m_wantedPosition);

    SmartDashboard.putNumber("PID Slot", m_wantedSlot);
    SmartDashboard.putNumber("Slot P=", m_armMotorPIDController.getP(m_wantedSlot));
    SmartDashboard.putNumber("Slot I=", m_armMotorPIDController.getI(m_wantedSlot));
    SmartDashboard.putNumber("Slot D=", m_armMotorPIDController.getD(m_wantedSlot));
    SmartDashboard.putNumber("Slot F=", m_armMotorPIDController.getFF(m_wantedSlot));
  }
}
