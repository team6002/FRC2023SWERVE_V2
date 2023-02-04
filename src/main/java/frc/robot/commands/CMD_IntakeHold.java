// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Intake;

public class CMD_IntakeHold extends CommandBase {
  SUB_Intake m_intake;
  public CMD_IntakeHold(SUB_Intake p_intake) {
    m_intake = p_intake;
  }

  @Override
  public void initialize() {
    m_intake.setHoldCurrent();
    if(m_intake.getIntakeState() == true){
      m_intake.setPower(.07);    
    }else{
      m_intake.setPower(-.07);
    }

  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
