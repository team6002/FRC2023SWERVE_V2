// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SUB_Climber;

public class CMD_ClimbDeployPrimaryClimber extends CommandBase {
  /** Initalizes primary arm and secondary arm to climbing position
   * (Primary arm at first bar position, secondary arm at full extension no spool)
   * primary: 80
   * secondary: 50
  */
  SUB_Climber m_climber;
  // private Timer m_runtime = new Timer();
  // private double m_maxtime = 1;
  // private boolean m_isFinished = false;
  public CMD_ClimbDeployPrimaryClimber(SUB_Climber p_climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = p_climber;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.initializePrimaryClimber();
    m_climber.setClimbing(true);
    // System.out.println("INITALIZE CLIMB MODE");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
