// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class LeftTrigger extends Trigger{
    RobotContainer mContainer = new RobotContainer();
    @Override
    public boolean get() {
        return mContainer.getDriveController().getLeftTriggerAxis() <= 0.3;
    }    
}

