// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.FSM_IntakeStatus;
import frc.robot.subsystems.SUB_Climber;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Turret;
import frc.robot.subsystems.FSM_IntakeStatus.IntakeState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_InitalizeClimbMode extends ParallelCommandGroup {
  /** 
   * Deploys the primary and secondary arms 
  */
  SUB_Climber m_climber;
  SUB_Turret m_turret;
  FSM_IntakeStatus m_intakeStatus;
  SUB_Intake m_intake;
  public CMD_InitalizeClimbMode(SUB_Climber p_climber, SUB_Turret p_turret, 
                            SUB_Intake p_intake, FSM_IntakeStatus p_intakeStatus) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_climber = p_climber;
    m_turret = p_turret;
    m_intake = p_intake;
    m_intakeStatus = p_intakeStatus;
    addCommands(
      new CMD_HopperOff(m_intake),
      new CMD_SetIntakeStatus(m_intakeStatus, IntakeState.HOME)
      ,new CMD_BackIntakeRetract(m_intake, m_intakeStatus)
      ,new CMD_FrontIntakeRetract(m_intake, m_intakeStatus)
      ,new CMD_ClimberSetClimb(m_climber, true),
      new CMD_TurretPrepareForClimb(m_turret)
      ,new CMD_ClimberPrimarySetHome(m_climber, false)
      ,new CMD_ClimberSecondarySetHome(m_climber, false)
      
    );
  }
    
}
