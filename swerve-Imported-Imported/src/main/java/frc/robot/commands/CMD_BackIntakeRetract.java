// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.FSM_IntakeStatus.IntakeState;
import frc.robot.subsystems.FSM_IntakeStatus;
public class CMD_BackIntakeRetract extends CommandBase {
  SUB_Intake m_Intake;
  FSM_IntakeStatus m_IntakeStatus;
  /** Creates a new CMD_BackIntakeToggle. */
  public CMD_BackIntakeRetract(SUB_Intake p_Intake, FSM_IntakeStatus p_IntakeStatus) {
    m_Intake = p_Intake;
    m_IntakeStatus = p_IntakeStatus;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

      m_Intake.setHopperOff();
      m_Intake.setBackIntakeOff();
      m_Intake.setBackIntakeRetract();


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
