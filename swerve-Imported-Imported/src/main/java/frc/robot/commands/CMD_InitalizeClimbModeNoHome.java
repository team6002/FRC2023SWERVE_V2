// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.SUB_Climber;
import frc.robot.subsystems.SUB_Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_InitalizeClimbModeNoHome extends ParallelCommandGroup {
  //init climb no homing, in case we miss
  SUB_Climber m_climber;
  SUB_Turret m_turret;
  public CMD_InitalizeClimbModeNoHome(SUB_Climber p_climber, SUB_Turret p_turret) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_climber = p_climber;
    m_turret = p_turret;
    addCommands(
      new CMD_ClimberSetClimb(m_climber, true),
      new CMD_TurretPrepareForClimb(m_turret),
      new CMD_ClimbDeployPrimaryClimber(m_climber),
      new CMD_ClimbDeploySecondaryClimber(p_climber)
    );
  }
}
