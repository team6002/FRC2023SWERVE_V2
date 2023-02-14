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
        // m_elbowMotor.setIdleMode(IdleMode.kBrake);
        m_elbowMotor.setIdleMode(IdleMode.kCoast);
        m_elbowMotorPIDController.setPositionPIDWrappingEnabled(false);
        m_elbowMotorPIDController.setOutputRange(-1, 1, 1);
        m_elbowMotorPIDController.setSmartMotionMaxVelocity(10, 1);
        m_elbowMotorPIDController.setSmartMotionMinOutputVelocity(-0, 1);
        m_elbowMotorPIDController.setSmartMotionMaxAccel(10, 1);
        m_elbowMotorPIDController.setSmartMotionAllowedClosedLoopError(1, 1);
        m_elbowMotorPIDController.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 1);
    }

    public void setReference(double p_reference){
        m_elbowMotorPIDController.setFF((Math.cos(Units.degreesToRadians(p_reference - 90))* 0.000445),1);
        m_elbowMotorPIDController.setReference(p_reference, ControlType.kPosition,1);
        m_wantedPosition = p_reference;
    }

    public double getElbowPosition(){
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

    double m_P = 0;//elbowConstants.kelbowP;
    double m_I = 0;//elbowConstants.kelbowI;
    double m_D = 0;//elbowConstants.kelbowD;
    double m_F = 0;//elbowConstants.kelbowF;
    
    public void telemetry(){

    SmartDashboard.putNumber("elbow position", m_elbowEncoder.getPosition());
    SmartDashboard.putNumber("elbow position(numeric)", getElbowPosition());

    // m_P = SmartDashboard.getNumber("P", m_P);
    // m_I = SmartDashboard.getNumber("I", m_I);
    // m_D = SmartDashboard.getNumber("D", m_D);
    // m_F = SmartDashboard.getNumber("F", m_F);
    // m_wantedPosition = SmartDashboard.getNumber("wantedPosition", m_wantedPosition);

    // SmartDashboard.putNumber("P", m_P);
    // SmartDashboard.putNumber("I", m_I);
    // SmartDashboard.putNumber("D", m_D);
    // SmartDashboard.putNumber("F", m_F);
    // SmartDashboard.putNumber("wantedPosition", m_wantedPosition);
   
    // m_elbowMotorPIDController.setP(m_P,1);
    // m_elbowMotorPIDController.setI(m_I,1);
    // m_elbowMotorPIDController.setD(m_D,1);
    // //m_elbowMotorPIDController.setFF(m_F,1);
    // m_elbowMotorPIDController.setFF(Math.cos(Units.degreesToRadians(m_wantedPosition-90))*m_F,1);
    // m_elbowMotorPIDController.setReference(m_wantedPosition, ControlType.kPosition, 1);

    // SmartDashboard.putNumber("velocity", m_elbowEncoder.getVelocity());
    // SmartDashboard.putNumber("output", m_elbowMotor.getAppliedOutput());
    // SmartDashboard.putNumber("wantedspeed", m_elbowMotor.get());
    // SmartDashboard.putNumber("elbow P", m_elbowMotorPIDController.getP());
    // SmartDashboard.putNumber("elbow I", m_elbowMotorPIDController.getI());
    // SmartDashboard.putNumber("elbow D", m_elbowMotorPIDController.getD());
    // SmartDashboard.putNumber("elbow F", m_elbowMotorPIDController.getFF(1));
    // SmartDashboard.putNumber("elbowSetpoint", m_wantedPosition);
    }
}
