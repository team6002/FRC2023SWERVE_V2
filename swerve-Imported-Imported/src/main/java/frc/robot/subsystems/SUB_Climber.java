// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class SUB_Climber extends SubsystemBase {
  private final Solenoid m_SecondSolenoid;
  private final Solenoid m_MainSolenoid;
 
  private CANSparkMax m_SecondaryClimberMotor2;
  private CANSparkMax m_SecondaryClimberMotor1;
  private CANSparkMax m_PrimaryClimberMotor2;
  private CANSparkMax m_PrimaryClimberMotor1;
 
  private SparkMaxLimitSwitch m_PrimaryHomeLimitSwitch;
  private SparkMaxLimitSwitch m_SecondaryHomeLimitSwitch;
  public boolean MainSolonoidState = false;
  public boolean SecondSolonoidState = false;
  
  private RelativeEncoder m_PrimaryEncoder;
  private RelativeEncoder m_SecondaryEncoder;

  private SparkMaxPIDController m_PrimaryClimberPID;
  private SparkMaxPIDController m_SecondaryClimberPID;

  private double PrimaryClimberSetpoint = 0;
  private double SecondaryClimberSetpoint = 0;
  private double m_previousPrimaryClimberSetpoint = 0;
  private double m_previousSecondaryClimberSetpoint = 0;
  private boolean climbing = false;
  private int m_primaryClimberDirection = 0;
  private int m_secondaryClimberDirection = 0;
  private int m_previousPrimaryClimberDirection = 0;
  private int m_previousSecondaryClimberDirection = 0;
  
  private int m_primaryClimberSmartMotionSlotID = 0;
  private int m_secondaryClimberSmartMotionSlotID = 0;
  /** Creates a new SUB_Climber. */
  public SUB_Climber() {
    
    m_SecondSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.kSecondSolonoid);
    m_MainSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.kMainSolonoid);
 
    m_SecondaryClimberMotor2 = new CANSparkMax(ClimberConstants.kSecondaryClimberMotor2,MotorType.kBrushless);
    m_SecondaryClimberMotor1 = new CANSparkMax(ClimberConstants.kSecondaryClimberMotor1,MotorType.kBrushless);
    m_PrimaryClimberMotor2 = new CANSparkMax(ClimberConstants.kPrimaryClimberMotor2,MotorType.kBrushless);
    m_PrimaryClimberMotor1 = new CANSparkMax(ClimberConstants.kPrimaryClimberMotor1,MotorType.kBrushless);
    
    m_PrimaryHomeLimitSwitch = m_PrimaryClimberMotor1.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_SecondaryHomeLimitSwitch = m_SecondaryClimberMotor1.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  
    m_PrimaryEncoder = m_PrimaryClimberMotor1.getEncoder();
    m_SecondaryEncoder = m_SecondaryClimberMotor1.getEncoder();

    m_PrimaryClimberPID = m_PrimaryClimberMotor1.getPIDController();
    m_SecondaryClimberPID = m_SecondaryClimberMotor1.getPIDController();
   
    m_PrimaryClimberMotor1.restoreFactoryDefaults();
    m_PrimaryClimberMotor2.restoreFactoryDefaults();
    m_SecondaryClimberMotor1.restoreFactoryDefaults();
    m_SecondaryClimberMotor2.restoreFactoryDefaults();

    m_PrimaryHomeLimitSwitch.enableLimitSwitch(true);
    m_SecondaryHomeLimitSwitch.enableLimitSwitch(true);
    
    m_SecondaryClimberMotor2.follow(m_SecondaryClimberMotor1);
    m_PrimaryClimberMotor2.follow(m_PrimaryClimberMotor1);

    m_SecondaryClimberMotor1.setInverted(true);

    m_PrimaryEncoder.setPositionConversionFactor(0.275);
    m_SecondaryEncoder.setPositionConversionFactor(0.297);

    m_SecondaryClimberMotor1.setIdleMode(IdleMode.kBrake);
    m_SecondaryClimberMotor2.setIdleMode(IdleMode.kBrake);
    m_PrimaryClimberMotor1.setIdleMode(IdleMode.kBrake);
    m_PrimaryClimberMotor2.setIdleMode(IdleMode.kBrake);

    // slot = 0 is ascending, slot 1 is descending
    // Primary PID for ascending
    m_PrimaryClimberPID.setFF(ClimberConstants.kPrimaryClimberFF,1);
    m_PrimaryClimberPID.setP(ClimberConstants.kPrimaryClimberP,1);
    m_PrimaryClimberPID.setI(ClimberConstants.kPrimaryClimberI,1);
    m_PrimaryClimberPID.setIZone(ClimberConstants.kPrimaryClimberIz,1);
    m_PrimaryClimberPID.setD(ClimberConstants.kPrimaryClimberD,1);
    m_PrimaryClimberPID.setSmartMotionMaxVelocity(ClimberConstants.kPrimaryClimberMaxVelocity, 1);
    m_PrimaryClimberPID.setSmartMotionMinOutputVelocity(0, 1);
    m_PrimaryClimberPID.setSmartMotionMaxAccel(ClimberConstants.kPrimaryClimberMaxAccel, 1);
    m_PrimaryClimberPID.setSmartMotionAllowedClosedLoopError(ClimberConstants.kPrimaryClimberAllowedError,1);
    m_PrimaryClimberPID.setOutputRange(ClimberConstants.kPrimaryClimberMinOutput,
                                       ClimberConstants.kPrimaryClimberMaxOutput,1);
    //Primary PID for descending
    m_PrimaryClimberPID.setFF(ClimberConstants.kPrimaryClimberFF2,2);
    m_PrimaryClimberPID.setP(ClimberConstants.kPrimaryClimberP2,2);
    m_PrimaryClimberPID.setI(ClimberConstants.kPrimaryClimberI2,2);
    m_PrimaryClimberPID.setIZone(ClimberConstants.kPrimaryClimberIz2,2);
    m_PrimaryClimberPID.setD(ClimberConstants.kPrimaryClimberD2,2);
    m_PrimaryClimberPID.setSmartMotionMaxVelocity(ClimberConstants.kPrimaryClimberMaxVelocity2, 2);
    m_PrimaryClimberPID.setSmartMotionMinOutputVelocity(0, 2);
    m_PrimaryClimberPID.setSmartMotionMaxAccel(ClimberConstants.kPrimaryClimberMaxAccel2, 2);
    m_PrimaryClimberPID.setSmartMotionAllowedClosedLoopError(ClimberConstants.kPrimaryClimberAllowedError2,2);
    m_PrimaryClimberPID.setOutputRange(ClimberConstants.kPrimaryClimberMinOutput2,
                                       ClimberConstants.kPrimaryClimberMaxOutput2,2);
    // Secondary PID for ascending
    m_SecondaryClimberPID.setFF(ClimberConstants.kSecondaryClimberFF,1);
    m_SecondaryClimberPID.setP(ClimberConstants.kSecondaryClimberP,1);
    m_SecondaryClimberPID.setI(ClimberConstants.kSecondaryClimberI,1);
    m_SecondaryClimberPID.setIZone(ClimberConstants.kSecondaryClimberIz,1);
    m_SecondaryClimberPID.setD(ClimberConstants.kSecondaryClimberD,1);
    m_SecondaryClimberPID.setSmartMotionMaxVelocity(ClimberConstants.kSecondaryClimberMaxVelocity, 1);
    m_SecondaryClimberPID.setSmartMotionMinOutputVelocity(0, 1);
    m_SecondaryClimberPID.setSmartMotionMaxAccel(ClimberConstants.kSecondaryClimberMaxAccel, 1);
    m_SecondaryClimberPID.setSmartMotionAllowedClosedLoopError(ClimberConstants.kSecondaryClimberAllowedError, 1);
    m_PrimaryClimberPID.setOutputRange(ClimberConstants.kSecondaryClimberMinOutput,
                                       ClimberConstants.kSecondaryClimberMaxOutput,1);
    // Secondary PID for descending
    m_SecondaryClimberPID.setFF(ClimberConstants.kSecondaryClimberFF2,2);
    m_SecondaryClimberPID.setP(ClimberConstants.kSecondaryClimberP2,2);
    m_SecondaryClimberPID.setI(ClimberConstants.kSecondaryClimberI2,2);
    m_SecondaryClimberPID.setIZone(ClimberConstants.kSecondaryClimberIz2,2);
    m_SecondaryClimberPID.setD(ClimberConstants.kSecondaryClimberD2,2);
    m_SecondaryClimberPID.setSmartMotionMaxVelocity(ClimberConstants.kSecondaryClimberMaxVelocity2, 2);
    m_SecondaryClimberPID.setSmartMotionMinOutputVelocity(0, 2);
    m_SecondaryClimberPID.setSmartMotionMaxAccel(ClimberConstants.kSecondaryClimberMaxAccel2, 2);
    m_SecondaryClimberPID.setSmartMotionAllowedClosedLoopError(ClimberConstants.kSecondaryClimberAllowedError2, 2);
    m_PrimaryClimberPID.setOutputRange(ClimberConstants.kSecondaryClimberMinOutput2,
                                       ClimberConstants.kSecondaryClimberMaxOutput2,2);

  }
  
  public void setClimbing(boolean isClimbing){
    climbing = isClimbing;
  }

  public boolean getClimbing(){
    return climbing;
  }

  public void setPrimaryEncoder(int pos){
    m_PrimaryEncoder.setPosition(pos);
  }

  public void setSecondaryEncoder(int pos){
    m_SecondaryEncoder.setPosition(pos);
  }

  public void setPrimaryGearDisengage(){
    // m_MainSolenoid.set(false);
    MainSolonoidState = false;
  }

  public void setPrimaryGearEngage(){
    // m_MainSolenoid.set(true);
    MainSolonoidState = true;
  }

  public void setSecondaryGearDisengage(){
    m_SecondSolenoid.set(false);
    SecondSolonoidState = false;
  }

  public void setSecondaryGearEngage(){
    m_SecondSolenoid.set(true);
    // SecondSolonoidState = true;
  }

  public void moveSecondaryClimber(double value){
    m_SecondaryClimberMotor1.set(value);
  }

  public void movePrimaryClimber(double value){
    m_PrimaryClimberMotor1.set(value);
  }
  public double getCurrentPrimaryPosition(){
    return m_PrimaryEncoder.getPosition();
  }

  public double getCurrentSecondaryPosition(){
    return m_SecondaryEncoder.getPosition();
  }


  public boolean getPrimaryHomeLimitSwitch(){
    return m_PrimaryHomeLimitSwitch.isPressed();
  }
  public boolean getSecondaryHomeLimitSwitch(){
    return m_SecondaryHomeLimitSwitch.isPressed();
  }
  public void initializePrimaryClimber(){
    // this is for initilizing climber faster
    m_PrimaryClimberPID.setReference(Constants.ClimberConstants.PrimaryClimberDeploy, ControlType.kSmartMotion,1);
  }
  public void initializeSecondaryClimber(){
    m_SecondaryClimberPID.setReference(Constants.ClimberConstants.SecondaryClimberDeploy, ControlType.kSmartMotion,1);
  }


  public void setPrimaryPosition(double p_wantedPosition){
    if (p_wantedPosition > getCurrentPrimaryPosition()){
        //Desecending
        // m_primaryClimberDirection = -1;
        m_primaryClimberSmartMotionSlotID = 2;
      }else{ 
        //Ascending
        // m_primaryClimberDirection = 1;
        m_primaryClimberSmartMotionSlotID = 1;
      }
      m_PrimaryClimberPID.setReference(p_wantedPosition, CANSparkMax.ControlType.kSmartMotion,m_primaryClimberSmartMotionSlotID);
      PrimaryClimberSetpoint = p_wantedPosition;
  }

  
  public void setSecondaryPosition(double p_wantedPosition){
    if (p_wantedPosition > getCurrentSecondaryPosition()){
      //Desecending
      // m_secondaryClimberDirection = -1;
      m_secondaryClimberSmartMotionSlotID = 2;
    }else{ 
      //Ascending
      // m_secondaryClimberDirection = 1;
      m_secondaryClimberSmartMotionSlotID = 1;
    }
    m_SecondaryClimberPID.setReference(p_wantedPosition, CANSparkMax.ControlType.kSmartMotion,m_secondaryClimberSmartMotionSlotID);
    SecondaryClimberSetpoint = p_wantedPosition;
  }
  //enableLimitSwitches and disableLimitSwitches should be done in pairs
  public void enableLimitSwitches(){
    m_PrimaryHomeLimitSwitch.enableLimitSwitch(true);
    m_SecondaryHomeLimitSwitch.enableLimitSwitch(true);
  }
  public void disableLimitSwitches(){
    m_PrimaryHomeLimitSwitch.enableLimitSwitch(false);
    m_SecondaryHomeLimitSwitch.enableLimitSwitch(false);
  }

  //ONLY USE FOR STARTUP, skips engaging the gears
  public void setPositionsOverride(double pos){
    PrimaryClimberSetpoint = pos;
    SecondaryClimberSetpoint = pos;
    m_PrimaryClimberPID.setReference(pos, ControlType.kSmartMotion);
    m_SecondaryClimberPID.setReference(pos, ControlType.kSmartMotion);
  }

  public double getPrimarySetpoint(){
    return PrimaryClimberSetpoint;
  }
  public double getSecondarySetpoint(){
    return SecondaryClimberSetpoint;
  }

  public double getPrimaryPosition(){
    return m_PrimaryEncoder.getPosition();
  }
  public double getSecondaryPosition(){
    return m_SecondaryEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // if (PrimaryClimberSetpoint != m_previousPrimaryClimberSetpoint){
    // m_PrimaryClimberPID.setReference(PrimaryClimberSetpoint, ControlType.kSmartMotion,m_primaryClimberSmartMotionSlotID);
    // }
    // SmartDashboard.putBoolean("isClimbing", getClimbing());
    // SmartDashboard.putBoolean("Primaryhomelimitswitch", getPrimaryHomeLimitSwitch());
    SmartDashboard.putNumber("PrimaryEncoder", getPrimaryPosition());
    SmartDashboard.putNumber("PrimaryVelocity", m_PrimaryEncoder.getVelocity());
    // SmartDashboard.putNumber("PrimaryAppliedOutput", m_PrimaryClimberMotor1.getAppliedOutput());
    
    // SmartDashboard.putBoolean("Secondaryhomelimitswitch", getSecondaryHomeLimitSwitch());
    SmartDashboard.putNumber("SecondaryEncoder", getSecondaryPosition());
    SmartDashboard.putNumber("SecnodaryVelocity", m_SecondaryEncoder.getVelocity());
    // SmartDashboard.putNumber("SecondaryAppliedOutput", m_SecondaryClimberMotor1.getAppliedOutput());

    // int Ticks = ThroughBore.get(); // 2000 ticks are about a rotation.
    // SmartDashboard.putNumber("ThroughBore", Ticks);
    // moveClimber(0.1);
    // // This method will be called once per schedule;r run
    m_previousPrimaryClimberSetpoint = PrimaryClimberSetpoint;
  }
}
