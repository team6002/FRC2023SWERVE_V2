// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.reflect.WildcardType;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SUB_Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_SafeAndSlowClimbFull extends SequentialCommandGroup {
  /** Creates a new CMD_ClimbFull. */
  SUB_Climber m_climber;
  public CMD_SafeAndSlowClimbFull(SUB_Climber p_climber) {

    m_climber = p_climber;
    addCommands(
      new CMD_ClimberPrimaryArmMove(m_climber, 0, 1.0), //Climber lift to 1st bar
      // new WaitCommand(0.15),
      new CMD_ClimberSecondaryArmMove(m_climber, 28.809, 1.0) //Climber captures 2nd bar

      ,new ParallelCommandGroup(
        new CMD_ClimberSecondaryArmMove(m_climber, 35, 2.0),
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          new CMD_ClimberPrimaryArmMove(m_climber, 17.0, 2.0)
        ) //The partial release
      )//major swing alert WEEEEEEEEEEEEE
      // new CMD_ClimberPrimaryArmMove(m_climber, 16.0, 2.0) //put primary arm behind 2nd bar, Surges somtimes
      ,new ParallelCommandGroup(
         new CMD_ClimberSecondaryArmMove(m_climber, 11.88, 1)//pull robot in
      ,new SequentialCommandGroup(
        new WaitCommand(1)
        ,new CMD_ClimberPrimaryArmMove(m_climber, 25.5, 1) //put primary arm behind 2nd bar
      )
      )
      ,new ParallelCommandGroup(
      new CMD_ClimberSecondaryArmMove(m_climber, 29.245,1) //lean primary arm against 2nd bar
      // new WaitCommand(0.1),
      ,new SequentialCommandGroup(
      new WaitCommand(1.5)
      ,new CMD_ClimberPrimaryArmMove(m_climber, 11, 1)  //swap hands on 2nd bar
      )
      )
      //REPEAT PARTIAL CLIMB
      ,new CMD_ClimberSecondaryArmMove(m_climber, 35, 1), //lay back the 2nd arm before repeating partial
      new CMD_ClimberPrimaryArmMove(m_climber, 0, 1), //Climber lift to 1st bar
      new CMD_ClimberSecondaryArmMove(m_climber, 28.809, 1.0) //Climber captures 2nd bar

      ,new ParallelCommandGroup(
        new CMD_ClimberSecondaryArmMove(m_climber, 35, 2.0),
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          new CMD_ClimberPrimaryArmMove(m_climber, 20.0, 2.0)
        ) //The partial release
      )//major swing alert WEEEEEEEEEEEEE
      
    );
  }
}