// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_LimeLight;

public class CMD_DriveAlignTag extends CommandBase {
  SUB_Drivetrain m_drivetrain;
  SUB_LimeLight m_limeLight;
  double x, z, yaw, xSpeed, zSpeed, rotSpeed;
  boolean m_isFinishedStrafe;
  boolean m_isFinishedTurn;
  boolean m_isFinishedDrive;

  public CMD_DriveAlignTag(SUB_Drivetrain p_drivetrain, SUB_LimeLight p_limeLight) {
    m_drivetrain = p_drivetrain;
    m_limeLight = p_limeLight;
    m_isFinishedDrive = true;
    m_isFinishedStrafe = false;
    m_isFinishedTurn = false;
    xSpeed = 0;
    zSpeed = 0;
    rotSpeed = 0;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if(m_limeLight.hasTarget()){
      x = m_limeLight.getTargetX();
      z = m_limeLight.getTargetZ();
      yaw = m_limeLight.getTargetYaw();

      if(Math.abs(x) > 1){
        xSpeed = Math.copySign(0.1, x);
        m_isFinishedStrafe = false;
      }else{
        xSpeed *= 0.4;
        m_isFinishedStrafe = true;
      }

      if(Math.abs(yaw) > 10 && m_isFinishedStrafe){
        rotSpeed = Math.copySign(0.1, yaw);
        m_isFinishedTurn = false;
      }else{
        rotSpeed *= .4;
        m_isFinishedTurn = true;
      }

      if(Math.abs(z) > 30){
        zSpeed = Math.copySign(0.2, z);
        m_isFinishedDrive = false;
      }else{
        zSpeed *= .4;
        m_isFinishedDrive = true;
      }
      
      m_drivetrain.drive(zSpeed, xSpeed, rotSpeed, false);
      
    }else{
      // m_drivetrain.drive(0, 0, 0, true);
    }
  }

  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return m_isFinishedStrafe && m_isFinishedTurn && m_isFinishedDrive;
  }
}
