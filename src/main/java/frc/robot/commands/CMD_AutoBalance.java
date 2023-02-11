// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_FiniteStateMachine;
import frc.robot.subsystems.SUB_FiniteStateMachine.RobotState;

public class CMD_AutoBalance extends CommandBase {
  boolean m_isFinished;
  SUB_Drivetrain m_drivetrain;
  SUB_FiniteStateMachine m_finiteStateMachine;
  public CMD_AutoBalance(SUB_Drivetrain p_drivetrain, SUB_FiniteStateMachine p_finiteStateMachine) {
    m_drivetrain = p_drivetrain;
    m_finiteStateMachine = p_finiteStateMachine;
    m_isFinished = false;
    m_finiteStateMachine.setState(RobotState.BALANCING);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double pitchError = 0 - m_drivetrain.getPitch();
    if(pitchError > 5){
      m_drivetrain.drive(0, 0.01, 0, false);
    }else if(pitchError < -5){
      m_drivetrain.drive(0, -0.01, 0, false);
    }else{
      m_drivetrain.drive(0, 0, 0, false);
      m_isFinished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
