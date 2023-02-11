// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_FiniteStateMachine.RobotState;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.SUB_FiniteStateMachine;

public class CMD_IntakeDrop extends CommandBase {
  SUB_Intake m_intake;
  SUB_FiniteStateMachine m_finiteStateMachine;


  public CMD_IntakeDrop(

    SUB_Intake p_intake,
    SUB_FiniteStateMachine p_finiteStateMachine
    ){
    m_intake = p_intake;
    m_finiteStateMachine = p_finiteStateMachine;
  }

  @Override
  public void initialize() {    
    m_intake.setIntakeCurrent();
    if(m_intake.getIntakeState() == false){
      m_intake.setPower(IntakeConstants.kIntakeDrop);
    }else{
      m_intake.setPower(-IntakeConstants.kIntakeDrop);
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}