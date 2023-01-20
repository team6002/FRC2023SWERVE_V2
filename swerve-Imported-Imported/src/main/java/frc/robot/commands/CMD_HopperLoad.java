// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Intake;

public class CMD_HopperLoad extends CommandBase {
  /** Checks if there is a ball in the external hoppers and then advances it
   * (Assume there is only one extra ball in hopper)
  */
  SUB_Intake m_intake;
  public CMD_HopperLoad(SUB_Intake p_intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = p_intake;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_intake.setIndexerOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setIndexerOff();
    if(m_intake.getFrontStatus()){
      m_intake.setFrontIntakeForward();
      m_intake.setHopperForward();
    }else {
      m_intake.setBackIntakeForward();
      m_intake.setHopperForward();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setHopperOff();
    m_intake.setFrontIntakeOff();
    m_intake.setBackIntakeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intake.getHopperStatus() || 
          (!m_intake.getFrontStatus() && !m_intake.getBackStatus());
  }
}
