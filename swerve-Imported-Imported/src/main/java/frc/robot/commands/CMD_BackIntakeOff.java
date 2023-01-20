// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Intake;

/** Add your docs here. */
public class CMD_BackIntakeOff extends CommandBase{
    private SUB_Intake m_Intake;

    public CMD_BackIntakeOff(SUB_Intake p_Intake){
        m_Intake = p_Intake;
    }

    @Override
    public void initialize() {
        m_Intake.setBackIntakeOff();
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}
