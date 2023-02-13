// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElbowConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SUB_FiniteStateMachine.RobotState;
import frc.robot.AUTO.*;


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
  public final SUB_Drivetrain m_robotDrive = new SUB_Drivetrain(m_blinkin, m_finiteStateMachine, m_limeLight);
  private final SUB_Intake m_intake = new SUB_Intake(m_finiteStateMachine, m_blinkin, m_limeLight);
  private final AUTO_Trajectory m_trajectory = new AUTO_Trajectory(m_robotDrive);
  private final BooleanSupplier IntakeToggle = () -> m_finiteStateMachine.getState() == RobotState.INTAKING;
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
    // m_elevator.setDefaultCommand(new CMD_ElevatorSetPower(m_elevator, m_driverControllerTrigger));
  }
  boolean pressed = false;
  private void configureButtonBindings() {

    // m_driverControllerTrigger.back().onTrue(new CMD_ElbowSetPosition(m_elbow, ElbowConstants.kElbowUp));


    // m_driverControllerTrigger.y().onTrue(new CMD_DriveAlignTag(m_robotDrive, m_limeLight));
    // m_driverControllerTrigger.x().onTrue(new CMD_DriveAlignLeft(m_robotDrive, m_limeLight));
    // m_driverControllerTrigger.b().onTrue(new CMD_DriveAlignRight(m_robotDrive, m_limeLight));

    m_driverControllerTrigger.leftBumper().onTrue(new ConditionalCommand((new CMD_HoldShelf(m_intake, m_elbow, m_elevator, m_wrist, m_finiteStateMachine)),
    new CMD_IntakeShelf(m_elbow, m_elevator, m_intake, m_wrist, m_finiteStateMachine), IntakeToggle));
    m_driverControllerTrigger.rightBumper().onTrue(new ConditionalCommand((new CMD_HoldGround(m_intake, m_elbow, m_elevator, m_wrist, m_finiteStateMachine)),
    new CMD_IntakeGround(m_elbow, m_elevator, m_intake, m_wrist, m_finiteStateMachine), IntakeToggle));

    m_driverControllerTrigger.y().onTrue(new CMD_PlaceThirdLevel(m_elevator, m_intake, m_elbow, m_wrist, m_finiteStateMachine));
    m_driverControllerTrigger.x().onTrue(new CMD_PlaceSecondLevel(m_elevator, m_intake, m_elbow, m_wrist, m_finiteStateMachine));
    m_driverControllerTrigger.a().onTrue(new CMD_PlaceGround(m_elevator, m_intake, m_elbow, m_wrist, m_finiteStateMachine));

    m_driverControllerTrigger.b().onTrue(new CMD_ToggleIntakeState(m_intake));
    // m_driverControllerTrigger.b().onTrue(new AUTO_Test(m_trajectory));
  }

    public void zeroGyroAngle() {
      m_robotDrive.zeroAngle();
    }

}