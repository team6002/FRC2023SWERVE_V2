// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FSM_IntakeStatus;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.FSM_IntakeStatus.IntakeState;

/** Add your docs here. */
public class CMD_FrontIntakeOff extends CommandBase{
    private SUB_Intake m_Intake;
    private FSM_IntakeStatus m_IntakeStatus;

    public CMD_FrontIntakeOff(SUB_Intake p_Intake){
        m_Intake = p_Intake;
        // m_IntakeStatus = p_IntakeStatus;
    }

    @Override
    public void initialize() {
        m_Intake.setFrontIntakeOff();
        // m_Intake.setFrontSolonoidRetract();
        // m_IntakeStatus.setState(IntakeState.HOME);
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}
