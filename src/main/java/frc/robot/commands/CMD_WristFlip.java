// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.SUB_Elbow;
import frc.robot.subsystems.SUB_Wrist;

public class CMD_WristFlip extends CommandBase {
  SUB_Elbow m_elbow;
  SUB_Wrist m_wrist;
  int m_side;//0 front, 1 back

  public CMD_WristFlip(SUB_Wrist p_wrist, SUB_Elbow p_elbow, int p_side) {
    m_wrist = p_wrist;
    m_elbow = p_elbow;
    m_side = p_side;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if(Math.abs(m_elbow.getElbowPosition() - ElbowConstants.kElbowUp) < 5){
      if(m_side == 0){
        m_wrist.setReference(WristConstants.kWristShelf);
      }else{
        m_wrist.setReference(WristConstants.kWristGround);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return m_wrist.checkPosition();
  }
}
