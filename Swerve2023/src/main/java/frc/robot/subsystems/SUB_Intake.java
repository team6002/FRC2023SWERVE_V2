// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.SUB_FiniteStateMachine.RobotState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.BlinkinConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Intake extends SubsystemBase {
    private final CANSparkMax m_intakeMotor;
    private final SparkMaxPIDController m_intakeMotorPIDController;
    private final DigitalInput m_sensor;
    private final SUB_FiniteStateMachine m_finiteStateMachine;
    private final SUB_Blinkin m_blinkin;
    private final SUB_LimeLight m_limeLight;
    private boolean m_intakeState = true;// true for cone mode, false for cube mode
  /** Creates a new SUB_Intake. */
  public SUB_Intake(SUB_FiniteStateMachine p_finiteStateMachine, SUB_Blinkin p_blinkin, SUB_LimeLight p_limeLight) {
    m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorCanID, MotorType.kBrushless);
    m_intakeMotorPIDController = m_intakeMotor.getPIDController();
    m_sensor = new DigitalInput(1);
    m_finiteStateMachine = p_finiteStateMachine;
    m_blinkin = p_blinkin;
    m_limeLight = p_limeLight;
    m_intakeMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setIntakeForward(){
    // m_intakeMotorPIDController.setReference(IntakeConstants.kIntakeForward, CANSparkMax.ControlType.kVelocity);
    m_intakeMotor.set(IntakeConstants.kIntakeForwardPower);
  }

  public void setIntakeOff(){
    m_intakeMotorPIDController.setReference(0, ControlType.kVelocity);
  }

  public void setIntakeReverse(){
    // m_intakeMotorPIDController.setReference(-IntakeConstants.kIntakeForward, CANSparkMax.ControlType.kVelocity);
    m_intakeMotor.set(-IntakeConstants.kIntakeForwardPower);
  }

  public boolean getSensor(){
    return m_sensor.get();
  }

  public void Off(){}

  public void setIntakeCurrent(){
    m_intakeMotor.setSmartCurrentLimit(35);
  }

  public void setHoldCurrent(){
    m_intakeMotor.setSmartCurrentLimit(5);
    // System.out.println(m_intakeMotor.getOutputCurrent());
  }

  public void setPower(double speed){
    m_intakeMotor.set(speed);
  }

  public void setIntakeState(boolean p_state){
    m_intakeState = p_state;
  }

  public boolean getIntakeState(){
    return m_intakeState;
  }

  @Override
  public void periodic(){
    // if we have a game piece, make the led strip sky blue colored
    if(m_finiteStateMachine.getState() != RobotState.SCORING && m_finiteStateMachine.getState() != RobotState.BALANCING){
      if(m_intakeState == true){
        if(m_finiteStateMachine.getState() == RobotState.INTAKING/* && getSensor()*/){
          m_blinkin.set(BlinkinConstants.kColor1Blink);
        }else{
          m_blinkin.set(BlinkinConstants.kYellow);
        }
      }else{
        if(m_finiteStateMachine.getState() == RobotState.INTAKING/* && getSensor()*/){
          m_blinkin.set(BlinkinConstants.kColor2Blink);
        }else{
          m_blinkin.set(BlinkinConstants.kPurple);
        }
      }
    }

    if(m_finiteStateMachine.getState() == RobotState.SCORING){
      if(m_intakeState == true){
        if(m_limeLight.hasTarget()){
            m_blinkin.set(BlinkinConstants.kYellow);
          }else{
            m_blinkin.set(BlinkinConstants.kColor1Blink);
          }
        }else{
          if(m_limeLight.hasTarget()){
            m_blinkin.set(BlinkinConstants.kColor2Chaser);
          }else{
            m_blinkin.set(BlinkinConstants.kPurple);
          }
        }
    }

    // if(m_finiteStateMachine.getState() == RobotState.INTAKING && getSensor()){
    //   m_finiteStateMachine.setState(RobotState.INTAKED);
    // }

    SmartDashboard.putNumber("Amps", m_intakeMotor.getOutputCurrent());
    SmartDashboard.putBoolean("intake state", m_intakeState);
  }
}
