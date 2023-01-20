
package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;
import frc.robot.autos.AUTO_Trajectory;
import frc.robot.commands.*;

/* 
  To do
  Make data logging that exports to a file.
  Fix the auto not turning properly
  make calibrating routine
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController m_driverController;
  private final XboxController m_operatorController;
  public final SwerveDrivetrain m_drivetrain = new SwerveDrivetrain();
  public final FSM_IntakeStatus m_intakeStatus = new FSM_IntakeStatus();
  public final SUB_Intake m_intake = new SUB_Intake(m_intakeStatus);
  public final SUB_Climber m_climber = new SUB_Climber();
  public final SUB_Turret m_turret;
  public final SUB_Shooter m_shooter = new SUB_Shooter();
  public final AUTO_Trajectory m_trajectory = new AUTO_Trajectory(m_drivetrain);
  Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driverController = new XboxController(0);
    m_operatorController = new XboxController(1);
    m_turret = new SUB_Turret(m_operatorController);

    // SmartDashboard.putData("SyncAngles", new CMD_SyncSwerveEncoders(m_drivetrain));
    SmartDashboard.putData("ResetAngles", new CMD_ResetSwerve(m_drivetrain));
    // SmartDashboard.putData("Secondary Climber Home", new CMD_ClimberSecondarySetHome(m_climber,true));
    // SmartDashboard.putData("Primary Climber Home", new CMD_ClimberPrimarySetHome(m_climber,true));
    // SmartDashboard.putData("Reset NavX", new CMD_ResetNavX(m_NavxGyro));
    // Configure the button bindings
    configureButtonBindings();
    m_drivetrain.setDefaultCommand(new SwerveDriveCommand(m_drivetrain, m_driverController));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new POVButton(m_operatorController, 90) //right d-pad
      .whenPressed(new CMD_SideTurret(m_turret));
    
    new POVButton(m_operatorController, 0) //up d-pad
      .whenPressed(new CMD_FrontTurret(m_turret));
    
    new POVButton(m_operatorController, 180) //down d-pad
      .whenPressed(new CMD_BackTurret(m_turret));
    
    new POVButton(m_operatorController, 270) //left d-pad
      .whenPressed(new CMD_ResetTurret(m_turret));

    new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value)
      .whenPressed(new CMD_TurretMode(m_turret));
    
    new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value)
      .whenPressed(new CMD_SetTurretJoystickMode(m_turret)); 

    // shooting
    new JoystickButton(m_operatorController, XboxController.Button.kA.value)
      .whenPressed(new CMD_Shooting(m_intake, m_intakeStatus, m_shooter));

    new JoystickButton(m_operatorController, XboxController.Button.kB.value)
      .whenPressed(new CMD_StopShooting(m_intake, m_intakeStatus, m_shooter)
    );

    new JoystickButton(m_operatorController, XboxController.Button.kBack.value)
      .whenPressed(new CMD_ClimberPrimarySetHome(m_climber, true)
    );

    new JoystickButton(m_operatorController, XboxController.Button.kStart.value)
      .whenPressed(new CMD_ClimberSecondarySetHome(m_climber, true)
    );

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
      .whenPressed(new CMD_ResetNavx(m_drivetrain)
    );

    new JoystickButton(m_driverController, XboxController.Button.kX.value)
      .whenPressed(new CMD_ClimberSecondarySolonoidExtend(m_climber)
    );

    // new JoystickButton(m_driverController, XboxController.Button.kB.value)
      // .whenPressed(new CMD_SpinInPlace(m_drivetrain, 90)
    // );

    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
      .whenPressed(new CMD_FrontIntakeToggle(m_intake, m_intakeStatus)
    );

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
      .whenPressed(new CMD_BackIntakeToggle(m_intake, m_intakeStatus)
    );

    new POVButton(m_driverController, 180)
      .whenPressed(new CMD_InitalizeClimbMode(m_climber, m_turret, m_intake, m_intakeStatus).withTimeout(4)
    );

    new POVButton(m_driverController, 0)
      .whenPressed(new CMD_InitalizeClimbModeNoHome(m_climber, m_turret).withTimeout(4)
    );

    new JoystickButton(m_driverController, XboxController.Button.kBack.value)
      .whenPressed(new CMD_ClimbPartial(m_climber)
    );

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
      .whenPressed(new CMD_SafeAndSlowClimbFull(m_climber)
    );

  }
  

  public XboxController getDriveController(){
    return m_driverController;
  }

  public XboxController getOperatorController() {
    return m_operatorController;
  }

  public double getRightX2() {
    return m_operatorController.getRightX();
  }



}
