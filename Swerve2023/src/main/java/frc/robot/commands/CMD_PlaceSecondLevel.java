// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SUB_Arm;
import frc.robot.subsystems.SUB_Intake;

public class CMD_PlaceSecondLevel extends SequentialCommandGroup {
  SUB_Arm m_arm;
  SUB_Intake m_intake;
  public CMD_PlaceSecondLevel(SUB_Arm p_arm, SUB_Intake p_intake) {
    m_arm = p_arm;
    m_intake = p_intake;
    
    addCommands(
      new CMD_ArmSetPosition(m_arm, 105),
      new CMD_ArmCheck(m_arm, 105),
      new CMD_DropItem(m_intake),
      new WaitCommand(1),
      new CMD_Stow(m_arm, m_intake)
    );
  }
}
