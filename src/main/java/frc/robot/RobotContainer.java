// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final SUB_Elbow m_elbow = new SUB_Elbow();
  private final SUB_Wrist m_wrist = new SUB_Wrist();
  private final SUB_Elevator m_elevator = new SUB_Elevator();
  private final SUB_Blinkin m_blinkin = new SUB_Blinkin();
  private final SUB_FiniteStateMachine m_finiteStateMachine = new SUB_FiniteStateMachine();
  private final SUB_LimeLight m_limeLight = new SUB_LimeLight(m_blinkin, m_finiteStateMachine);
  private final SUB_Drivetrain m_robotDrive = new SUB_Drivetrain(m_blinkin, m_finiteStateMachine, m_limeLight);
  private final SUB_Intake m_intake = new SUB_Intake(m_finiteStateMachine, m_blinkin, m_limeLight);
  // The driver's controller
  XboxController m_operatorController = new XboxController(1);
  CommandXboxController m_driverControllerTrigger = new CommandXboxController(0);
  Trigger xButton = m_driverControllerTrigger.leftBumper();

  /**
   * The container for the robot. Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(new CMD_DriveCommand(m_robotDrive, m_driverControllerTrigger));
  }
  boolean pressed = false;
  private void configureButtonBindings() {


    // m_driverControllerTrigger.b().onTrue(new SequentialCommandGroup(//cancel
    //  new CMD_ElevatorSetOff(m_elevator),
    //  new CMD_IntakeOff(m_intake)
    // ));

    // m_driverControllerTrigger.a().onTrue(new CMD_ToggleIntakeState(m_intake));

    m_driverControllerTrigger.leftTrigger().onTrue(new CMD_IntakeShelf(m_elbow, m_elevator, m_intake, m_wrist))
    .onFalse(new CMD_Hold(m_intake, m_elbow, m_elevator, m_wrist));

    m_driverControllerTrigger.rightTrigger().onTrue(new CMD_IntakeGround(m_elbow, m_elevator, m_intake, m_wrist))
    .onFalse(new CMD_Hold(m_intake, m_elbow, m_elevator, m_wrist));

    m_driverControllerTrigger.leftBumper().onTrue(new CMD_PlaceThirdLevel(m_elevator, m_intake, m_elbow, m_wrist, m_finiteStateMachine));
   
    m_driverControllerTrigger.rightBumper().onTrue(new CMD_PlaceSecondLevel(m_elevator, m_intake, m_elbow, m_wrist, m_finiteStateMachine));
   
    // m_driverControllerTrigger.leftTrigger().whileTrue(new InstantCommand(() -> m_elevator.setElevatorOn())).
    // onFalse(new InstantCommand(() -> m_elevator.setElevatorOff()));

    // m_driverControllerTrigger.rightTrigger().whileTrue(new InstantCommand(() -> m_elevator.setElevatorReverse())).
    // onFalse(new InstantCommand(() -> m_elevator.setElevatorOff()));

    // m_driverControllerTrigger.leftBumper().whileTrue(new InstantCommand(() -> m_intake.setIntakeForward())).
    // onFalse(new InstantCommand(() -> m_intake.Off()));

    // m_driverControllerTrigger.rightBumper().whileTrue(new InstantCommand(() -> m_intake.setIntakeReverse())).
    // onFalse(new InstantCommand(() -> m_intake.Off()));

    // m_driverControllerTrigger.a().whileTrue(new InstantCommand(() -> m_wrist.setForward())).
    // onFalse(new InstantCommand(() -> m_wrist.setOff()));

    // m_driverControllerTrigger.b().whileTrue(new InstantCommand(() -> m_wrist.setReverse())).
    // onFalse(new InstantCommand(() -> m_wrist.setOff()));

    // m_driverControllerTrigger.x().whileTrue(new InstantCommand(() -> m_elbow.setReverse())).
    // onFalse(new InstantCommand(() -> m_elbow.setOff()));

    // m_driverControllerTrigger.y().whileTrue(new InstantCommand(() -> m_elbow.setForward())).
    // onFalse(new InstantCommand(() -> m_elbow.setOff()));

  }

    public void zeroGyroHeading() {
      m_robotDrive.zeroHeading();
    }

}