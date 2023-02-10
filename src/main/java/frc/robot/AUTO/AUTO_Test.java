// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.AUTO;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AUTO_Test extends SequentialCommandGroup {
  AUTO_Trajectory m_trajectory;
  public AUTO_Test(AUTO_Trajectory p_trajectory) {
    m_trajectory = p_trajectory;

    addCommands(
      new SequentialCommandGroup(
        m_trajectory.driveTrajectory(m_trajectory.testTrajectory)
      )
  );
  }
}
