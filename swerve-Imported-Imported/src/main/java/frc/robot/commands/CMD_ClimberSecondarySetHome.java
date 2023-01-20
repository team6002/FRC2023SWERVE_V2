// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Climber;

public class CMD_ClimberSecondarySetHome extends CommandBase {
  /** Creates a new CMD_ClimberSecondarySetHome. */
  SUB_Climber m_climber;
  private boolean m_disengageOnCompletion;
  public CMD_ClimberSecondarySetHome(SUB_Climber p_climber, boolean p_disengageOnCompletion) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = p_climber;
    m_disengageOnCompletion = p_disengageOnCompletion;
    // addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_climber.setSecondaryGearEngage();
    m_climber.moveSecondaryClimber(-0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println("FINISHED HOMING SECONDARY CLIMBER");
    m_climber.setSecondaryEncoder(0); //reset encoder
    m_climber.moveSecondaryClimber(0); //stop moving
    // if(m_disengageOnCompletion){
    //   m_climber.setSecondaryGearDisengage(); //latch
    // }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climber.getSecondaryHomeLimitSwitch();
  }
}
