package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.lib.util.linearInterpolator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SUB_Shooter extends SubsystemBase{
    //motors
    private CANSparkMax m_ShooterMaster = new CANSparkMax(ShooterConstants.kShooterMaster, MotorType.kBrushless);
    private CANSparkMax m_ShooterSlave = new CANSparkMax(ShooterConstants.kShooterSlave, MotorType.kBrushless);

    //encoders
    private RelativeEncoder m_ShooterMasterEncoder = m_ShooterMaster.getEncoder();
    // private RelativeEncoder m_ShooterSlaveEncoder = m_ShooterSlave.getEncoder();

    //PID controller
    private SparkMaxPIDController m_Controller = m_ShooterMaster.getPIDController();

    private double m_ShooterSetpoint = ShooterConstants.kShootingVelocity;
    
    private linearInterpolator m_ShooterInterpolator;
    private boolean wantShooter = false;
    private double m_targetDistance;
    private boolean m_autoMode; // used in AUTO
    private double m_autoShooterSetpoint = 1000; // Used to set Setpoint in AUTO
    //Network Table
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");


    public SUB_Shooter()
    {
        m_ShooterMaster.restoreFactoryDefaults();
        m_ShooterSlave.restoreFactoryDefaults();

        m_ShooterMaster.setIdleMode(IdleMode.kCoast);
        m_ShooterSlave.setIdleMode(IdleMode.kCoast);

        m_ShooterSlave.follow(m_ShooterMaster, true);
        m_ShooterMaster.setInverted(true);
        m_Controller.setFF(ShooterConstants.kShooterFF);
        m_Controller.setP(ShooterConstants.kShooterP);
        m_Controller.setI(ShooterConstants.kShooterI);
        m_Controller.setD(ShooterConstants.kShooterD);

        m_Controller.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
        m_Controller.setSmartMotionMaxVelocity(ShooterConstants.kShootingVelocity, 0);
        m_Controller.setSmartMotionMaxAccel(ShooterConstants.kShootingAccel, 0);
        SmartDashboard.putNumber("Desired Shooter Setpoint", ShooterConstants.kShootingVelocity);
       
        m_ShooterInterpolator = new linearInterpolator(ShooterConstants.kShooterArray);
    }

    public void setAutoShooterSetpoint(double p_wantedSetpoint)
    {
    m_autoShooterSetpoint = p_wantedSetpoint;
    }

    public void setAutoMode(boolean mode){
        m_autoMode = mode;
    }

    //turns off shooter
    public void shooterOff()
    {
        wantShooter = false;
        // m_ShooterMaster.set(0);
    }

    //gets the shooter up to speed
    public void readyShooter()
    {
        wantShooter = true;
        // m_Controller.setReference(ShooterConstants.kShootingVelocity, CANSparkMax.ControlType.kVelocity);
    }

    public double getDistance() {
        // double y = 0.0;
        try {
            m_targetDistance = table.getEntry("ty").getDouble(0.0);
        }
        catch(Exception e) {
            
        }

        return m_targetDistance;
    }


    //checks if the shooter is ready to shoot
    public boolean isReady(double setpoint, double epsilon)
    {
        return (getVelocity() - epsilon <= setpoint) && (getVelocity() + epsilon >= setpoint);
    }

    //gets velocity of shooter
    public double getVelocity()
    {
        return m_ShooterMasterEncoder.getVelocity();
    }
    public double getShooterSetpoint(){
        return m_ShooterSetpoint;
    }

    @Override
    public void periodic() {
        //must press tab to set in smartdashboard
        // m_ShooterSetpoint = SmartDashboard.getNumber("Desired Shooter Setpoint", 
        //                                                 ShooterConstants.kShootingVelocity);
        m_targetDistance = getDistance();                                                
        if(wantShooter){
            /* Twisted Devil's field 
            2.65 with front bumper on the tarmac.
            
            */
            // m_Controller.setReference(m_ShooterSetpoint, ControlType.kVelocity);
            m_ShooterSetpoint = m_ShooterInterpolator.getInterpolatedValue(m_targetDistance);
            // if (m_autoMode){
                // m_ShooterSetpoint = 1000;
            // }else{
            // m_ShooterSetpoint = (m_targetDistance*-30)+3000;
            // }
            m_Controller.setReference(m_ShooterSetpoint, ControlType.kVelocity);
        }else{
            m_Controller.setReference(0, ControlType.kDutyCycle);
        }
        
        SmartDashboard.putNumber("targetDistance", m_targetDistance);
        SmartDashboard.putBoolean("Shooting", wantShooter);
        SmartDashboard.putNumber("ShooterVelocity", getVelocity());
        SmartDashboard.putNumber("Interpolated value", m_ShooterSetpoint);
    }
}
