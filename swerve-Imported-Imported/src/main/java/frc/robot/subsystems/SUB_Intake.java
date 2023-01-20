// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.FSM_IntakeStatus.IntakeState;



/** Add your docs here. */
public class SUB_Intake extends SubsystemBase {
    private FSM_IntakeStatus m_intakeStatus;

    public CANSparkMax m_FrontIntakeMotor= new CANSparkMax(IndexerConstants.kFrontIntake,MotorType.kBrushless);
    public CANSparkMax m_BackIntakeMotor= new CANSparkMax(IndexerConstants.kBackIntake,MotorType.kBrushless);
    private CANSparkMax m_HopperMotor= new CANSparkMax(IndexerConstants.kHopper,MotorType.kBrushless);
    private CANSparkMax m_IndexerMotor= new CANSparkMax(IndexerConstants.kIndexer,MotorType.kBrushless);
   
    public DigitalInput m_FrontIntakeSensor = new DigitalInput(IndexerConstants.kFrontIntakeIR);
    private DigitalInput m_BackIntakeSensor = new DigitalInput(IndexerConstants.kBackIntakeIR);
    private DigitalInput m_HopperSensor = new DigitalInput(IndexerConstants.kHopperIR);

    private RelativeEncoder m_FrontIntakeEncoder = m_FrontIntakeMotor.getEncoder();
    private RelativeEncoder m_BackIntakeEncoder = m_BackIntakeMotor.getEncoder();
  
    
    private final Solenoid m_FrontIntakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, IndexerConstants.kFrontIntakeSolonoid);
    private final Solenoid m_BackIntakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, IndexerConstants.kBackIntakeSolonoid);

    public double frontIntakeState = 0;
    public double backIntakeState = 0;
    public double indexerState = 0;
    public double hopperState = 0;
    public boolean previousHopperStatus = false;
    public boolean previousFrontIntakeStatus = false;
    public boolean previousBackIntakeStatus = false;
    public boolean frontIntakeDeployed = false;
    public boolean backIntakeDeployed = false;
    public boolean previousFrontIntakeDeployed= false;
    public boolean previousBackIntakeDeployed= false;
    private SparkMaxPIDController m_FrontController = m_FrontIntakeMotor.getPIDController();
    private SparkMaxPIDController m_BackController = m_BackIntakeMotor.getPIDController();    
    private SparkMaxPIDController m_HopperController = m_HopperMotor.getPIDController();
    private SparkMaxPIDController m_IndexerController = m_IndexerMotor.getPIDController();
    
    public SUB_Intake(FSM_IntakeStatus p_IntakeStatus) {
      m_intakeStatus = p_IntakeStatus;
        
      m_HopperMotor.setInverted(true);
      m_IndexerMotor.setInverted(true);

      // m_FrontIntakeMotor.setSmartCurrentLimit(IndexerConstants.kIntakeCurrentLimit);
      // m_BackIntakeMotor.setSmartCurrentLimit(IndexerConstants.kIntakeCurrentLimit);
      // m_HopperMotor.setSmartCurrentLimit(IndexerConstants.kHopperCurrentLimit);

      m_FrontController.setFF(IndexerConstants.kIntakeFF);
      m_FrontController.setP(IndexerConstants.kIntakeP);
      m_FrontController.setI(IndexerConstants.kIntakeI);
      m_FrontController.setD(IndexerConstants.kIntakeD);

      m_FrontController.setOutputRange(IndexerConstants.kMinOutput, IndexerConstants.kMaxOutput);
      m_FrontController.setSmartMotionMaxVelocity(IndexerConstants.kIntakeVelocity, 0);
      m_FrontController.setSmartMotionMaxAccel(IndexerConstants.kIntakeAccel, 0);

      m_BackController.setFF(IndexerConstants.kIntakeFF);
      m_BackController.setP(IndexerConstants.kIntakeP);
      m_BackController.setI(IndexerConstants.kIntakeI);
      m_BackController.setD(IndexerConstants.kIntakeD);

      m_BackController.setOutputRange(IndexerConstants.kMinOutput, IndexerConstants.kMaxOutput);
      m_BackController.setSmartMotionMaxVelocity(IndexerConstants.kIntakeVelocity, 0);
      m_BackController.setSmartMotionMaxAccel(IndexerConstants.kIntakeAccel, 0);

      m_HopperController.setFF(IndexerConstants.kHopperFF);
      m_HopperController.setP(IndexerConstants.kHopperP);
      m_HopperController.setI(IndexerConstants.kHopperI);
      m_HopperController.setD(IndexerConstants.kHopperD);

      m_HopperController.setOutputRange(IndexerConstants.kMinOutput, IndexerConstants.kMaxOutput);
      m_HopperController.setSmartMotionMaxVelocity(IndexerConstants.kHopperVelocity, 0);
      m_HopperController.setSmartMotionMaxAccel(IndexerConstants.kHopperAccel, 0);
      
      m_IndexerController.setFF(IndexerConstants.kIndexerFF);
      m_IndexerController.setP(IndexerConstants.kIndexerP);
      m_IndexerController.setI(IndexerConstants.kIndexerI);
      m_IndexerController.setD(IndexerConstants.kIndexerD);

      m_IndexerController.setOutputRange(IndexerConstants.kMinOutput, IndexerConstants.kMaxOutput);
      m_IndexerController.setSmartMotionMaxVelocity(IndexerConstants.kIndexerVelocity, 0);
      m_IndexerController.setSmartMotionMaxAccel(IndexerConstants.kIndexerAccel, 0);
    }
    
  public void setFrontIntakeForward(){
    m_FrontController.setReference(IndexerConstants.kIntakeVelocity, CANSparkMax.ControlType.kVelocity);
    frontIntakeState = 1;
  }
  public void setFrontIntakeReverse(){
    m_FrontController.setReference(-IndexerConstants.kIntakeVelocity, CANSparkMax.ControlType.kVelocity);
    frontIntakeState = -1;
  }
  public void setFrontIntakeOff(){
    m_FrontIntakeMotor.set(IndexerConstants.IntakeOff);
    frontIntakeState = 0;
  }
  public double getFrontVelocity(){
      return m_FrontIntakeEncoder.getVelocity();
  }
  public void setFrontIntakeExtend(){
    m_FrontIntakeSolenoid.set(true);
    frontIntakeDeployed = true;
  }
  public void setFrontIntakeRetract(){
    m_FrontIntakeSolenoid.set(false);
    frontIntakeDeployed = false;
  }
  public boolean getFrontStatus(){
      return m_FrontIntakeSensor.get();     
  }
  public void setBackIntakeForward(){
    m_BackController.setReference(IndexerConstants.kIntakeVelocity, CANSparkMax.ControlType.kVelocity);
    backIntakeState = 1;
  }
  public void setBackIntakeReverse(){
    m_BackController.setReference(-IndexerConstants.kIntakeVelocity, CANSparkMax.ControlType.kVelocity);
    backIntakeState = -1;
  }
  public void setBackIntakeOff(){
    m_BackIntakeMotor.set(IndexerConstants.IntakeOff);
    backIntakeState = 0;
  }
  public double getBackVelocity(){
      return m_BackIntakeEncoder.getVelocity();
  }
  public void setBackIntakeExtend(){
    m_BackIntakeSolenoid.set(true);
    backIntakeDeployed = true;
  }
  public void setBackIntakeRetract(){
    m_BackIntakeSolenoid.set(false);
    backIntakeDeployed = false;
  }
  public boolean getBackStatus(){
    return m_BackIntakeSensor.get();
  }
  public void setHopperOff(){
    m_HopperMotor.set(0);
    hopperState = 0;
  }
  public void setHopperForward(){
    // m_HopperMotor.set(1); //4500
    m_HopperController.setReference(IndexerConstants.kHopperVelocity, CANSparkMax.ControlType.kVelocity);
    hopperState = 1;
  }

  public void setHopperBackward(){
    m_HopperController.setReference(-IndexerConstants.kHopperVelocity, CANSparkMax.ControlType.kVelocity);
    hopperState = -1;
  }
  
  public void setIndexerOff(){
    m_IndexerMotor.set(IndexerConstants.IndexerOff);
    indexerState = 0;
  }
  
  public void setIndexerForward(){
    m_IndexerController.setReference(IndexerConstants.kIndexerVelocity, CANSparkMax.ControlType.kVelocity);
    indexerState = 1;
  }

  public void setIndexerBackward(){
    m_IndexerController.setReference(-IndexerConstants.kIndexerVelocity, CANSparkMax.ControlType.kVelocity);
    indexerState = -1;
  }

  public boolean getHopperStatus(){
    return m_HopperSensor.get();
  }

  public boolean isFrontIntaking(){
    return frontIntakeState == 1;
  }

  public boolean isBackIntaking(){
    return backIntakeState == 1;
  }

  public boolean isFrontDeployed(){
    return frontIntakeDeployed;
  }

  public boolean isBackDeployed(){
    return backIntakeDeployed;
  }


  @Override
  public void periodic() {
    if(m_intakeStatus.isState(IntakeState.INTAKE)){
      if(getHopperStatus()){
       setHopperOff();
      }else
        if(getBackStatus()||getFrontStatus()){
        setHopperForward();
      }else {
        setHopperOff();
      }
    
    if (isFrontDeployed()){
      if(getHopperStatus()){
        if(getFrontStatus()){
          setFrontIntakeOff();
        }else setFrontIntakeForward();
      }else{
        setFrontIntakeForward();
      }
    }else {
    setFrontIntakeOff();
    }

    if (isBackDeployed()){
      if(getHopperStatus()){
        if(getBackStatus()){
          setBackIntakeOff();
        }else setBackIntakeForward();
      }else {
       setBackIntakeForward();
    }
    }else {
      setBackIntakeOff();
    }

    }else if(m_intakeStatus.isState(IntakeState.SHOOTING)) {
      
    }
    
    if (isBackDeployed() && isFrontDeployed()){
      if (previousFrontIntakeDeployed){
        setFrontIntakeRetract();
        setFrontIntakeOff();
        frontIntakeState = 0;
      }else
      {  
      setBackIntakeRetract();
      setBackIntakeOff();
      backIntakeState = 0;
      }
    }
    previousHopperStatus = getHopperStatus();
    previousFrontIntakeStatus = getFrontStatus();
    previousBackIntakeStatus = getBackStatus();
    previousFrontIntakeDeployed = isFrontDeployed();
    previousBackIntakeDeployed = isBackDeployed();
    SmartDashboard.putBoolean("FrontIntake?Full?", getFrontStatus());
    SmartDashboard.putBoolean("BackIntake?Full?", getBackStatus());
    SmartDashboard.putBoolean("HopperFull?", getHopperStatus());

    SmartDashboard.putBoolean("Front Deployed", isFrontDeployed());
    SmartDashboard.putBoolean("Back Deployed", isBackDeployed());
    SmartDashboard.putBoolean("Front Intake On", isFrontIntaking());
    SmartDashboard.putBoolean("Back Intake On", isBackIntaking());

    SmartDashboard.putString("IntakeState", m_intakeStatus.getCurrentState().toString());
    SmartDashboard.putNumber("Hopper Velocity", m_HopperMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("FrontIntake Velocity", m_FrontIntakeEncoder.getVelocity());
    SmartDashboard.putNumber(("BackIntakeVelocity"), m_BackIntakeEncoder.getVelocity());
    SmartDashboard.putNumber("Indexer Velocity", m_IndexerMotor.getEncoder().getVelocity());
  }
}
        