// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_LimeLight;

public class CMD_DriveAlignTagRight extends CommandBase {
  SUB_Drivetrain m_drivetrain;
  SUB_LimeLight m_limeLight;
  double x, y, yaw, xSpeed, ySpeed, rotSpeed, xError, yError, yawError, x_p, y_p, yaw_p, timer;
  boolean m_isFinishedStrafe;
  boolean m_isFinishedTurn;
  boolean m_isFinishedDrive;

  public CMD_DriveAlignTagRight(SUB_Drivetrain p_drivetrain, SUB_LimeLight p_limeLight) {
    m_drivetrain = p_drivetrain;
    m_limeLight = p_limeLight;
    m_isFinishedDrive = true;
    m_isFinishedStrafe = false;
    m_isFinishedTurn = false;
    xSpeed = 0;
    ySpeed = 0;
    rotSpeed = 0;
    x_p = .0125 * 1.5;
    y_p = .0033 * 1.5;
    yaw_p = .0025 * 1.5;
    timer = 0;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    ySpeed *= .2;
    rotSpeed *= .2;
    xSpeed *= 0.2;

    if(m_limeLight.hasTarget()){
      y = m_limeLight.getTargetY() - 21;
      x = m_limeLight.getTargetX();
      yaw = m_limeLight.getTargetYaw();
      yawError = 0 + yaw;
      xError = 0 + x;
      yError = 0 + y;
      
      if(Math.abs(x) > 25){
        xSpeed = xError * x_p;
        m_isFinishedDrive = false;
      }else{
        m_isFinishedDrive = true;
      }

      if(Math.abs(y) < 1){
        ySpeed = yError * y_p;
        m_isFinishedStrafe = false;
      }else{
        m_isFinishedStrafe = true;
      }

      if(Math.abs(yaw) > 2.5){
        rotSpeed = yawError * yaw_p;
        m_isFinishedTurn = false;
      }else{
        m_isFinishedTurn = true;
      }
      
      m_drivetrain.drive(xSpeed, ySpeed, rotSpeed, false);
      timer = 0;
      
    }else{
      timer += .02;
    }
    SmartDashboard.putNumber("autodrive x", x);
  }

  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return m_isFinishedStrafe && m_isFinishedTurn && m_isFinishedDrive || timer > 2;
  }
}
