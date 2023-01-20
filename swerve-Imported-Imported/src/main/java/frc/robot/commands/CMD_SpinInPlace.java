// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class CMD_SpinInPlace extends CommandBase {
  /** Creates a new CMD_SpinInPlace. */
  SwerveDrivetrain m_drivetrain;
  double m_angle;
  public CMD_SpinInPlace(SwerveDrivetrain p_drivetrain, double p_angle) {
    m_drivetrain = p_drivetrain;
    m_angle = p_angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Turning to angle");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.drive(0, 0, 0.5, true);
    System.out.println("Turning to angle");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("didn't make it  to angle");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean gotToAngle =  (m_drivetrain.getDegrees() <= m_angle + 2.5 && m_drivetrain.getDegrees() >= m_angle - 2.5 );
    System.out.println(m_drivetrain.getDegrees());
    System.out.println(gotToAngle);
    return gotToAngle;
  
  }
}
