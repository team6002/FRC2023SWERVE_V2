// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


public class SUB_FiniteStateMachine {

    public enum RobotState 
    {
        HOME,
        INTAKING,
        INTAKED,
        SCORING,
        BALANCING,
    }

    private RobotState m_currentState = RobotState.BALANCING;

    public void setState(RobotState p_State) {
        m_currentState = p_State;
    }
    
    public RobotState getState() {
        return m_currentState;
    }
    
    public boolean isState(RobotState p_State) {
        return (m_currentState == p_State);
    }

    public RobotState getCurrentState(){
        return m_currentState;
    }

}