// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_LimeLight;

public class CMD_DriveAlignTag extends CommandBase {
  SUB_Drivetrain m_drivetrain;
  SUB_LimeLight m_limeLight;
  double x, y, yaw, xSpeed, ySpeed, rotSpeed, xError, yError, yawError, x_p, y_p, yaw_p, timer,x_f,y_f,yaw_f;
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
    ySpeed = 0;
    rotSpeed = 0;
    x_p = 0.005;
    x_f = 0.05;
    y_p = .007;
    y_f = 0.05;
    yaw_p = .0025;
    yaw_f = 0.05;
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
      y = m_limeLight.getTargetY();
      x = m_limeLight.getTargetX();
      yaw = m_limeLight.getTargetYaw();
      yawError = 0 + yaw;
      xError = 0 + x;
      yError = 0 + y;
      
      if(Math.abs(x) > 30){
        xSpeed = (xError * x_p)+ Math.copySign(x_f,x);
        m_isFinishedDrive = false;
      }else{
        m_isFinishedDrive = true;
      }

      if(Math.abs(y) > 1){
        ySpeed = (yError * y_p)+ Math.copySign(y_f,y);
        m_isFinishedStrafe = false;
      }else{
        m_isFinishedStrafe = true;
      }

      if(Math.abs(yaw) > 4){
        rotSpeed = (yawError * yaw_p)+ Math.copySign(yaw_f,yaw);
        m_isFinishedTurn = false;
      }else{
        m_isFinishedTurn = true;
      }
      
      m_drivetrain.drive(xSpeed, ySpeed, rotSpeed, false);
      timer = 0;
      }else{
      timer += .02;
    }
  }

  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    m_drivetrain.setOdometryPositionInches(x, y, yaw);
    return m_isFinishedStrafe && m_isFinishedTurn && m_isFinishedDrive || timer > 2;
  }
}
