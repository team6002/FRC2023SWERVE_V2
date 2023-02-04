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
import frc.robot.Constants.ElbowConstants;
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
    }

    public void setReference(double p_reference){
        m_elbowMotorPIDController.setReference(p_reference, ControlType.kPosition);
        m_wantedPosition = p_reference;
    }

    public double getElbowPosition(){
        return m_elbowEncoder.getPosition();
    }

    public double getElbowWantedPosition(){
        return m_wantedPosition;
    }

    public boolean checkPosition(){
        if(m_wantedPosition == getElbowPosition()){
            return true;
        }else{
            return false;
        }
    }

    @Override
    public void periodic() {
        telemetry();
    }

    public void telemetry(){
        SmartDashboard.putNumber("elbow position", m_elbowEncoder.getPosition());
        SmartDashboard.putNumber("elbow position(numeric)", getElbowPosition());
    }
}
