// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.FSM_IntakeStatus;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.subsystems.FSM_IntakeStatus.IntakeState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_StopShooting extends ParallelCommandGroup {
  /** Stops emptying our hopper and switches the intake state to neutral(IntakeState.Home) */
  
  SUB_Intake m_intake;
  FSM_IntakeStatus m_intakeStatus;
  SUB_Shooter m_shooter;
  public CMD_StopShooting(SUB_Intake p_intake, FSM_IntakeStatus p_intakeStatus, SUB_Shooter p_shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_intake = p_intake;
    m_intakeStatus = p_intakeStatus;
    m_shooter = p_shooter;
    addCommands(
      new CMD_ShooterOff(m_shooter),
      new CMD_HopperOff(m_intake),
      new CMD_IndexerOff(m_intake)
      // new CMD_BackIntakeOff(m_intake),
      // new CMD_FrontIntakeOff(m_intake),
      // new CMD_SetIntakeStatus(m_intakeStatus, IntakeState.HOME)
      // new CMD_BackSolonoidRetract(m_intake),
      // new CMD_FrontSolonoidRetract(m_intake),
    );
  }
}
