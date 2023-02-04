// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Arm;

public class CMD_ArmSetOff extends CommandBase {
  SUB_Arm m_arm;
  public CMD_ArmSetOff(SUB_Arm p_arm) {
    m_arm = p_arm;
  }

  @Override
  public void initialize() {
    m_arm.setArmOff();
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
