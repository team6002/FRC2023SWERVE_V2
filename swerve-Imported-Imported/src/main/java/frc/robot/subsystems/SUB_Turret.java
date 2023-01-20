package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class SUB_Turret extends SubsystemBase{
    //motors, encoders, PID controller, limit switches
    private CANSparkMax m_Turret = new CANSparkMax(TurretConstants.kTurretMotor, MotorType.kBrushless);
    private final RelativeEncoder m_Encoder = m_Turret.getEncoder();
    private SparkMaxPIDController m_Controller = m_Turret.getPIDController();
    private SparkMaxLimitSwitch m_ForwardLimitSwitch = m_Turret.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    private SparkMaxLimitSwitch m_ReverseLimitSwitch = m_Turret.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    private double center = 27; //the limelight x val goes from -27 to 27
    public int huntDirection = 1; //goes positive initially
    private double targetPosition = Math.toRadians(90); //-Math.PI/2; //-90;
    double validAngle = 0;

    //turretMode: auto = 0; mannual = 1; reset = -1; joystick = 2
    private int turretMode = 0;
    private final double RESET_TURRET = Math.toRadians(-140); //value of encoder when left (reverse) limit switch is triggered

    //Network Table
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    //joystick
    XboxController joystick;

    //ontarget (global bool)

    public SUB_Turret(XboxController p_joystick){
        joystick = p_joystick;

        m_Turret.setIdleMode(IdleMode.kBrake);
        m_Turret.setInverted(false);

        m_Controller.setFF(TurretConstants.kTurretFF); //NO FF FOR POSITION
        m_Controller.setP(TurretConstants.kTurretP);
        m_Controller.setI(TurretConstants.kTurretI);
        m_Controller.setD(TurretConstants.kTurretD);

        m_Controller.setOutputRange(TurretConstants.kMinTurretOutput, TurretConstants.kMaxTurretOutput);
        m_Controller.setSmartMotionMaxAccel(TurretConstants.kMaxTurretAccel, 0);
        m_Controller.setSmartMotionMaxVelocity(TurretConstants.kMaxTurretVelocity, 0);

        m_ForwardLimitSwitch.enableLimitSwitch(true);
        m_ReverseLimitSwitch.enableLimitSwitch(true);

        m_Turret.setIdleMode(IdleMode.kCoast);

        m_Encoder.setPositionConversionFactor((2*Math.PI) / 30); //converts position to radian
        m_Encoder.setVelocityConversionFactor((2*Math.PI) / 30); //converts velocity to radian
        m_Encoder.setPosition(Math.PI/2); //starting position of turret during auto (rad)
    }

    //Reads from the network table (x and y val is how far the camera is from the target)
    public double readtX() { 
        double x = 0.0;
        try {
            x = table.getEntry("tx").getDouble(0.0);
        }
        catch(Exception e) {
            
        }
        
        return -x;
    }

    public double readtY() {
        double y = 0.0;
        try {
            y = table.getEntry("ty").getDouble(0.0);
        }
        catch(Exception e) {
            
        }
        
        return y;
    }

    //if the limelight sees any targets
    public double readtV() {
        double v = 0.0;
        try {
            v = table.getEntry("tv").getDouble(0.0);
        }
        catch(Exception e) {
            
        }
        
        return v;
    }

    //calculate how far the target is from center
    //right is negative, left is positive (Camera)
    public double diffFromCenter() {
        return readtX() - center;
    }

    public void turretReset() {
        turretMode = -1;
    }

    public void setFrontPosition() {
        targetPosition = -Math.PI/2; //-90;
    }

    public void setBackPosition() {
        targetPosition = Math.PI/2; //90;
    }

    public void setSidePosition() {
        targetPosition = 0;
    }

    public void setTurretMode(int wantedMode) {
        turretMode = wantedMode;
    }

    public int getTurretMode() {
        return turretMode;
    }
    
    //sets the which way the turret should turn to find a target
    public void setHuntDirection(int dir) {
        if(dir == 1) {
            huntDirection = 1;
        }
        else {
            huntDirection = -1;
        }
    }

    public double validateAngle(double p_angle) {
        if(Math.abs(p_angle) < Math.abs(RESET_TURRET)) {
            return p_angle;
        }
        return Math.copySign(RESET_TURRET, p_angle);
    }

    public void setReferenceAngle(double referenceAngleRadians) {

        // double currentAngleRadians = m_Encoder.getPosition();
    
        // double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        // if (currentAngleRadiansMod < 0.0) {
        //   currentAngleRadiansMod += 2.0 * Math.PI;
        // }
    
        // // The reference angle has the range [0, 2pi) but the Neo's encoder can go above that
        // double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        // if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
        //   adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        // } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
        //   adjustedReferenceAngleRadians += 2.0 * Math.PI;
        // }
    
        // targetPosition = referenceAngleRadians;
        
        SmartDashboard.putNumber("Desired Turret Position", Math.toDegrees(referenceAngleRadians));
            
        m_Controller.setReference(referenceAngleRadians, ControlType.kSmartMotion);
    }
    
    @Override
    public void periodic() {
        // double sentOutput = 0;

        if(turretMode == 0) { //auto mode = 0
            // targetPosition = 0;
            if(readtV() == 0) { //no target found

            }
            else {
                //algo
                /*
                get degree offset from limelight, add to current pos, check if position is valid
                set smart controller to vaild pos 
                */
                //gets wanted position
                double newAngle = Math.toRadians(readtX()) + m_Encoder.getPosition() - Math.toRadians(8); //0.5 offset

                validAngle = validateAngle(newAngle);
                setReferenceAngle(validAngle);
            }
        }
        else if (turretMode == 1) { //manual position mode
            validAngle =  validateAngle(targetPosition);
            setReferenceAngle(validAngle);
        }
        else if(turretMode == -1) { //reset turret encoder and moves turret to back
            if(m_ReverseLimitSwitch.isPressed() == true) {
                m_Turret.setVoltage(0);
                m_Encoder.setPosition(RESET_TURRET); //resets encoder
                validAngle = validateAngle(-Math.PI/2); //goes to front position
                setReferenceAngle(validAngle);
                turretMode = 0;
            } else {
                m_Turret.setVoltage(-1); //goes towards forward limit switch
            }
        } 
        else if(turretMode == 2) { //joystick mode
            // m_Turret.setVoltage(-1);
            // double xVal = joystick.getLeftX();
            // if(Math.abs(xVal) < 0.1) {
            //     xVal = 0;
            // }
            // sentOutput = xVal * TurretConstants.kTurretJoystickVoltage;
        }

        //Shuffleboard Output
        SmartDashboard.putNumber("Turret X", readtX());
        SmartDashboard.putNumber("Turret Y", readtY());
        SmartDashboard.putNumber("Targets?", readtV());
        // SmartDashboard.putNumber("Voltage", sentOutput);
        // SmartDashboard.putNumber("Difference", diffFromCenter);
        // SmartDashboard.putBoolean("Target?", onTarget);
        // SmartDashboard.putNumber("Hunting Direction", huntDirection);
        SmartDashboard.putBoolean("Forward Limit Switch", m_ForwardLimitSwitch.isPressed());
        SmartDashboard.putBoolean("Reverse Limit Switch", m_ReverseLimitSwitch.isPressed());
        SmartDashboard.putNumber("Turret Encoder", m_Encoder.getPosition());
        SmartDashboard.putNumber("Valid Position", validAngle);
        SmartDashboard.putNumber("Target Encoder", targetPosition);
        SmartDashboard.putNumber("Turret Mode", turretMode);
        SmartDashboard.putNumber("Turret Velocity", m_Encoder.getVelocity());
        // SmartDashboard.putNumber("Turret Velocity Conversion", m_Encoder.getVelocityConversionFactor());

        // SmartDashboard.putBoolean("Forward Enabled", m_ForwardLimitSwitch.isLimitSwitchEnabled());
        // SmartDashboard.putBoolean("Reverse Enabled", m_ReverseLimitSwitch.isLimitSwitchEnabled());
    }

    //soft limit forward = 43
    //soft limit reverse = -16
}