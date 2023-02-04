// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.BlinkinConstants;

public class SUB_Blinkin extends SubsystemBase {
  Spark m_blinkin;

  public SUB_Blinkin() {
    m_blinkin = new Spark(0);
  }

  public void setHasTarget(){
    m_blinkin.set(BlinkinConstants.kGreen);
  }

  public void setHasNoTarget(){
    m_blinkin.set(BlinkinConstants.kRedStrobe);
  }

  public void setHasNoGamePiece(){
    m_blinkin.set(BlinkinConstants.kRed);
  }

  public void setCelebrate(){
    m_blinkin.set(BlinkinConstants.kFireLarge);
  }

  public void setFrontFar(){
    m_blinkin.set(BlinkinConstants.kTippedFrontFar);
  }

  public void setFront(){
    m_blinkin.set(BlinkinConstants.kTippedFront);
  }

  public void setBackFar(){
    m_blinkin.set(BlinkinConstants.kTippedBack);
  }

  public void setBack(){
    m_blinkin.set(BlinkinConstants.kTippedBackFar);
  }

  public void set(double p_pwm){
    m_blinkin.set(p_pwm);
  }

  @Override
  public void periodic() {
  }
}
