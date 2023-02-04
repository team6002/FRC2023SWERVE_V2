// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Intake;

public class CMD_IntakeSetState extends CommandBase {
  SUB_Intake m_intake;
  boolean m_intakeState;
  public CMD_IntakeSetState(SUB_Intake p_intake, boolean p_intakeState) {
    m_intake = p_intake;
    m_intakeState = p_intakeState;
  }

  @Override
  public void initialize() {
    m_intake.setIntakeState(m_intakeState);
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
