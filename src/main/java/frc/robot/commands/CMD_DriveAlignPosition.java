// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_LimeLight;
//Note this does not generate a trajectory but heads straight for a spot, only use this for aligning for targets
public class CMD_DriveAlignPosition extends CommandBase {
  SUB_Drivetrain m_drivetrain;
  SUB_LimeLight m_limeLight;
  double x, y, yaw,
    xSpeed, ySpeed, rotSpeed,
    xError, yError, yawError,
    x_p, y_p, yaw_p, timer,
    x_f, y_f, yaw_f,
    x_target, y_target, yaw_target;
  boolean m_isFinishedStrafe;
  boolean m_isFinishedTurn;
  boolean m_isFinishedDrive;

  public CMD_DriveAlignPosition(SUB_Drivetrain p_drivetrain, double p_x, double p_y, double P_yaw) {// feed in target location
    m_drivetrain = p_drivetrain;
    m_isFinishedDrive = true;
    m_isFinishedStrafe = false;
    m_isFinishedTurn = false;
    xSpeed = 0;
    ySpeed = 0;
    rotSpeed = 0;
    x_p = 0.007;
    x_f = 0.05;
    y_p = .007;
    y_f = 0.05;
    yaw_p = .0025;
    yaw_f = 0.05;
    timer = 0;
    x_target = p_x;
    y_target = p_y;
    yaw_target = P_yaw;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    ySpeed *= .2;
    rotSpeed *= .2;
    xSpeed *= 0.2;
    x = Units.metersToInches(m_drivetrain.getX());
    y = Units.metersToInches(m_drivetrain.getY());
    yaw = m_drivetrain.getAngle();
    xError = x_target - x;
    yError = y_target - y;
    yawError = yaw_target - yaw;
    
    if(Math.abs(xError) > 2){
      xSpeed = (xError * x_p)+ Math.copySign(x_f,xError);
      m_isFinishedDrive = false;
    }else{
      m_isFinishedDrive = true;
    }

    if(Math.abs(yError) > 2){
      ySpeed = (yError * y_p)+ Math.copySign(y_f,yError);
      m_isFinishedStrafe = false;
    }else{
      m_isFinishedStrafe = true;
    }

    if(Math.abs(yawError) > 4){
      rotSpeed = (yawError * yaw_p)+ Math.copySign(yaw_f,yawError);
      m_isFinishedTurn = false;
    }else{
      m_isFinishedTurn = true;
    }
    
    m_drivetrain.drive(xSpeed, ySpeed, rotSpeed, false);
  }

  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return m_isFinishedStrafe && m_isFinishedTurn && m_isFinishedDrive;
  }
}
