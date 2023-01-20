package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_ShooterOn extends CommandBase {
    SUB_Shooter m_Shooter;
    private Timer m_runtime = new Timer();
    private double m_maxRuntime = 3; // 3 second
    private double m_shooterConfirms = 0;

    public CMD_ShooterOn(SUB_Shooter p_Shooter)
    {
        m_Shooter = p_Shooter;
    }
  
  
    @Override
    public void initialize() {
      m_shooterConfirms = 0;
      m_Shooter.readyShooter();
      m_runtime.reset();
    }

    @Override
    public void execute(){
      if (m_Shooter.getVelocity() >= m_Shooter.getShooterSetpoint() &&
       m_Shooter.getVelocity() <= m_Shooter.getShooterSetpoint()+200){
        m_shooterConfirms += 1;
      }
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        boolean isFinish = (m_shooterConfirms == 3);
        if (m_runtime.get() > m_maxRuntime) isFinish = true;
        
        return isFinish;
    }
}
