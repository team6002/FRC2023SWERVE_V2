// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import frc.robot.Constants.ElbowConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Elbow extends SubsystemBase {

    private final CANSparkMax m_elbowMotor;
    private final SparkMaxPIDController m_elbowMotorPIDController;
    private final AbsoluteEncoder m_elbowEncoder;
    double m_wantedPosition;

    public SUB_Elbow() {
        m_elbowMotor = new CANSparkMax(ElbowConstants.kElbowMotorCanID, MotorType.kBrushless);
        m_elbowMotorPIDController = m_elbowMotor.getPIDController();
        m_elbowEncoder = m_elbowMotor.getAbsoluteEncoder(Type.kDutyCycle);
        m_elbowEncoder.setPositionConversionFactor(360);
        m_elbowEncoder.setVelocityConversionFactor(6);
        m_elbowEncoder.setInverted(true);
        m_elbowMotorPIDController.setP(ElbowConstants.kElbowP,1);
        m_elbowMotorPIDController.setI(ElbowConstants.kElbowI,1);
        m_elbowMotorPIDController.setD(ElbowConstants.kElbowD,1);
        m_elbowMotorPIDController.setFF(ElbowConstants.kElbowF,1);
        m_elbowMotorPIDController.setFeedbackDevice(m_elbowEncoder);
        m_elbowMotor.setIdleMode(IdleMode.kCoast);
        m_elbowMotorPIDController.setPositionPIDWrappingEnabled(false);
        m_elbowMotorPIDController.setOutputRange(ElbowConstants.kElbowMinOutput, ElbowConstants.kElbowMaxOutput, 1);
        m_elbowMotorPIDController.setSmartMotionMaxVelocity(10, 1);
        m_elbowMotorPIDController.setSmartMotionMinOutputVelocity(-0, 1);
        m_elbowMotorPIDController.setSmartMotionMaxAccel(10, 1);
        m_elbowMotorPIDController.setSmartMotionAllowedClosedLoopError(1, 1);
        m_elbowMotorPIDController.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 1);
   
        // SmartDashboard.putNumber("Elbow P", ElbowConstants.kElbowP);
        // SmartDashboard.putNumber("Elbow I", ElbowConstants.kElbowI);
        // SmartDashboard.putNumber("Elbow D", ElbowConstants.kElbowD);
        // SmartDashboard.putNumber("Elbow F", ElbowConstants.kElbowF);
    }

    public void setReference(double p_reference){
        m_wantedPosition = p_reference;
        m_elbowMotorPIDController.setFF((Math.cos(Units.degreesToRadians(p_reference - 90))* ElbowConstants.kElbowF),1);
        m_elbowMotorPIDController.setReference(p_reference, ControlType.kPosition,1);
    }

    public double getPosition(){
        return m_elbowEncoder.getPosition();
    }

    public double getElbowWantedPosition(){
        return m_wantedPosition;
    }

    public void setReverse(){
        m_elbowMotor.set(-.1);
    }

    public void setForward(){
        m_elbowMotor.set(.1);
    }

    public void setOff(){
        m_elbowMotor.set(0);
    }

    // public boolean checkPosition(){
    //     if(m_wantedPosition == getElbowPosition()){
    //         return true;
    //     }else{
    //         return false;
    //     }
    // }

    @Override
    public void periodic() {
        telemetry();
    }

    // double m_P = ElbowConstants.kElbowP;
    // double m_I = ElbowConstants.kElbowI;
    // double m_D = ElbowConstants.kElbowD;
    // double m_F = ElbowConstants.kElbowF;
    // double m_minOutput = 0;
    // double m_maxOutput = 0;
    public void telemetry(){

      SmartDashboard.putNumber("elbow position", m_elbowEncoder.getPosition());
      SmartDashboard.putNumber("elbow position(numeric)", getPosition());

    //   m_P = SmartDashboard.getNumber("Elbow P", m_P);
    //   m_I = SmartDashboard.getNumber("Elbow I", m_I);
    //   m_D = SmartDashboard.getNumber("Elbow D", m_D);
    //   m_F = SmartDashboard.getNumber("Elbow F", m_F);
    //   m_minOutput = SmartDashboard.getNumber("Min Elbow Output", m_minOutput);
    //   m_maxOutput = SmartDashboard.getNumber("Max Elbow Output", m_maxOutput);
        // m_wantedPosition = SmartDashboard.getNumber("wantedPosition", m_wantedPosition);

    //   SmartDashboard.putNumber("Elbow P", m_P);
    //   SmartDashboard.putNumber("Elbow I", m_I);
    //   SmartDashboard.putNumber("Elbow D", m_D);
    //   SmartDashboard.putNumber("Elbow F", m_F);
    //   SmartDashboard.putNumber("Min Elbow Output", m_minOutput);
    //   SmartDashboard.putNumber("Max Elbow Output", m_maxOutput);
      // SmartDashboard.putNumber("wantedPosition", m_wantedPosition);
    
    //   m_elbowMotorPIDController.setP(m_P,1);
    //   m_elbowMotorPIDController.setI(m_I,1);
    //   m_elbowMotorPIDController.setD(m_D,1);
    //   m_elbowMotorPIDController.setFF(Math.cos(Units.degreesToRadians(m_wantedPosition-90))*m_F,1);
    //   m_elbowMotorPIDController.setOutputRange(m_minOutput, m_maxOutput, 1);
      // m_elbowMotorPIDController.setReference(m_wantedPosition, ControlType.kPosition, 1);

      // SmartDashboard.putNumber("velocity", m_elbowEncoder.getVelocity());
      // SmartDashboard.putNumber("output", m_elbowMotor.getAppliedOutput());
      // SmartDashboard.putNumber("wantedspeed", m_elbowMotor.get());
      SmartDashboard.putNumber("elbow P", m_elbowMotorPIDController.getP(1));
      SmartDashboard.putNumber("elbow I", m_elbowMotorPIDController.getI(1));
      SmartDashboard.putNumber("elbow D", m_elbowMotorPIDController.getD(1));
      SmartDashboard.putNumber("elbow F", m_elbowMotorPIDController.getFF(1));
      SmartDashboard.putNumber("elbowSetpoint", m_wantedPosition);
    }
}
