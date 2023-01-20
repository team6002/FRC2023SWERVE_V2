package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_ShooterOff extends CommandBase {
    SUB_Shooter m_Shooter;

    public CMD_ShooterOff(SUB_Shooter p_Shooter)
    {
        m_Shooter = p_Shooter;
    }

    @Override
    public void initialize() {
        m_Shooter.shooterOff();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
