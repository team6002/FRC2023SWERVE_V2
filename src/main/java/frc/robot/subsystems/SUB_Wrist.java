// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import frc.robot.Constants.WristConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Wrist extends SubsystemBase {

    private final CANSparkMax m_wristMotor;
    private final SparkMaxPIDController m_wristMotorPIDController;
    private final AbsoluteEncoder m_wristEncoder;
    double m_wantedPosition;
    double m_tolerance = 5;

    public SUB_Wrist() {
      m_wristMotor = new CANSparkMax(WristConstants.kWristMotorCanID, MotorType.kBrushless);
      m_wristMotorPIDController = m_wristMotor.getPIDController();
      m_wristEncoder = m_wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
      m_wristMotor.setInverted(false);
      m_wristEncoder.setPositionConversionFactor(360);
      m_wristEncoder.setVelocityConversionFactor(6);
      m_wristEncoder.setInverted(false);
      // m_wristMotor.setSoftLimit(null, 80);
      m_wristMotorPIDController.setP(WristConstants.kWristP,1);
      m_wristMotorPIDController.setI(WristConstants.kWristI,1);
      m_wristMotorPIDController.setD(WristConstants.kWristI,1);
      m_wristMotorPIDController.setFF(WristConstants.kWristF,1);
      m_wristMotorPIDController.setFeedbackDevice(m_wristEncoder);
      m_wristMotorPIDController.setPositionPIDWrappingEnabled(false);
      m_wristMotorPIDController.setOutputRange(-1, 1, 1);
      m_wristMotorPIDController.setSmartMotionMaxVelocity(10, 1);
      m_wristMotorPIDController.setSmartMotionMinOutputVelocity(-0, 1);
      m_wristMotorPIDController.setSmartMotionMaxAccel(5, 1);
      m_wristMotorPIDController.setSmartMotionAllowedClosedLoopError(1, 1);
      m_wristMotorPIDController.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 1);
    
      SmartDashboard.putNumber("Wrist P", WristConstants.kWristP);
      SmartDashboard.putNumber("Wrist I", WristConstants.kWristI);
      SmartDashboard.putNumber("Wrist D", WristConstants.kWristD);
      SmartDashboard.putNumber("Wrist F", WristConstants.kWristF);
    }


    public void setReference(double p_reference){
      m_wristMotorPIDController.setReference(p_reference, ControlType.kPosition,1);
      m_wantedPosition = p_reference;
    }

    public double getWristPosition(){
      return m_wristEncoder.getPosition();
    }

    public void setReverse(){
      m_wristMotor.set(-1);
    }

    public void setOff(){
      m_wristMotor.set(0);
    }

    public void setForward(){
      m_wristMotor.set(1);
    }

    @Override
    public void periodic() {
      telemetry();
    }

    public boolean checkPosition(){
      if(Math.abs(getWristPosition() - m_wantedPosition) < m_tolerance){
        return true;
      }else{
        return false;
      }
    }
    double m_P,m_I,m_D,m_F;
    public void telemetry(){
      SmartDashboard.putNumber("wrist position", m_wristEncoder.getPosition());
      SmartDashboard.putNumber("wrist position(numeric)", getWristPosition());
  
      m_P = SmartDashboard.getNumber("Wrist P", m_P);
      m_I = SmartDashboard.getNumber("Wrist I", m_I);
      m_D = SmartDashboard.getNumber("Wrist D", m_D);
      m_F = SmartDashboard.getNumber("Wrist F", m_F);
      // m_wantedPosition = SmartDashboard.getNumber("wantedPosition", m_wantedPosition);
  
      SmartDashboard.putNumber("Wrist P", m_P);
      SmartDashboard.putNumber("Wrist I", m_I);
      SmartDashboard.putNumber("Wrist D", m_D);
      SmartDashboard.putNumber("Wrist F", m_F);
      // SmartDashboard.putNumber("wantedPosition", m_wantedPosition);
     
      m_wristMotorPIDController.setP(m_P,1);
      m_wristMotorPIDController.setI(m_I,1);
      m_wristMotorPIDController.setD(m_D,1);
      m_wristMotorPIDController.setFF(m_F,1);
      // m_wristMotorPIDController.setReference(m_wantedPosition, ControlType.kPosition, 1);
  
      // SmartDashboard.putNumber("velocity", m_wristEncoder.getVelocity());
      // SmartDashboard.putNumber("output", m_wristMotor.getAppliedOutput());
      // SmartDashboard.putNumber("wantedspeed", m_wristMotor.get());
      // SmartDashboard.putNumber("wrist P", m_wristMotorPIDController.getP(1));
      // SmartDashboard.putNumber("wrist I", m_wristMotorPIDController.getI(1));
      // SmartDashboard.putNumber("wrist D", m_wristMotorPIDController.getD(1));
      // SmartDashboard.putNumber("wrist F", m_wristMotorPIDController.getFF(1));
      SmartDashboard.putNumber("Wrist Setpoint", m_wantedPosition);
    }
}
