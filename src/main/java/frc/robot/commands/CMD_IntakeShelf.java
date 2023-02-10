// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.SUB_Elbow;
import frc.robot.subsystems.SUB_Elevator;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_IntakeShelf extends SequentialCommandGroup {
  public CMD_IntakeShelf(SUB_Elbow p_elbow, SUB_Elevator p_elevator, SUB_Intake p_intake, SUB_Wrist p_wrist) {
    addRequirements(p_elbow, p_elevator, p_wrist, p_wrist);
    addCommands(
      new CMD_ElbowSetPosition(p_elbow, ElbowConstants.kElbowUp),//ground position
      new CMD_WristFlip(p_wrist, p_elbow, 0),
      new CMD_ElevatorSetPosition(p_elevator, ElevatorConstants.kElevatorShelf),
      new CMD_ElevatorCheck(p_elevator, ElevatorConstants.kElevatorShelf),
      new CMD_ElbowSetPosition(p_elbow, ElbowConstants.kElbowForwards),
      new CMD_IntakeOn(p_intake)
    );
  }
}
