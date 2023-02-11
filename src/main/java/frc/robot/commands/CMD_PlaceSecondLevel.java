// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.SUB_Elbow;
import frc.robot.subsystems.SUB_Elevator;
import frc.robot.subsystems.SUB_FiniteStateMachine;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Wrist;
import frc.robot.subsystems.SUB_FiniteStateMachine.RobotState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_PlaceSecondLevel extends SequentialCommandGroup {
  /** Creates a new CMD_PlaceThirdLevel. */
  SUB_Elevator m_elevator;
  SUB_Intake m_intake;
  SUB_Elbow m_elbow;
  SUB_Wrist m_wrist;
  SUB_FiniteStateMachine m_finiteStateMachine;
  public CMD_PlaceSecondLevel(SUB_Elevator p_elevator, SUB_Intake p_intake, SUB_Elbow p_elbow, SUB_Wrist p_wrist, SUB_FiniteStateMachine p_finiteStatemachine) {
    m_elevator = p_elevator;
    m_intake = p_intake;
    m_elbow = p_elbow;
    m_wrist = p_wrist;
    m_finiteStateMachine = p_finiteStatemachine;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(m_elevator, m_intake, m_elbow, m_wrist, m_finiteStateMachine);
    addCommands(
      new CMD_setState(m_finiteStateMachine, RobotState.SCORING),
      new CMD_ElbowSetPosition(m_elbow, ElbowConstants.kElbowLift),
      new CMD_ElevatorSetPosition(m_elevator, ElevatorConstants.kElevatorPrep),
      new CMD_ElevatorCheck(m_elevator, ElevatorConstants.kElevatorPrep),
      new ParallelCommandGroup(
        new CMD_ElevatorSetPosition(p_elevator, ElevatorConstants.kElevatorSecondLevel),
        new CMD_ElbowSetPosition(p_elbow, ElbowConstants.kElbowForwards),
        new CMD_WristSetPosition(m_elbow, m_wrist, WristConstants.kWristShelf)
      ),
      new CMD_ElevatorCheck(p_elevator, ElevatorConstants.kElevatorSecondLevel),
      new CMD_IntakeDrop(m_intake, m_finiteStateMachine),
      new WaitCommand(1),
      new CMD_Stow(m_elevator, m_intake, m_elbow, m_wrist)
    );
  }
}
