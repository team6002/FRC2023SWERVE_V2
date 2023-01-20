// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Climber;
public class CMD_ClimberPrimaryToggle extends CommandBase {
  /** Creates a new CMD_ClimberExtend. */
  SUB_Climber m_climber;
  public CMD_ClimberPrimaryToggle(SUB_Climber p_climber) {
    m_climber = p_climber;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!m_climber.MainSolonoidState){
      m_climber.setPrimaryGearEngage();
    }else{ 
      m_climber.setPrimaryGearDisengage();
    }
    System.out.println("TOOGLE");
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
