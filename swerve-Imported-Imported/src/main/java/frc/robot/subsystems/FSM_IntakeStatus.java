// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

public class FSM_IntakeStatus {

    public enum IntakeState 
    {
        HOME,
        INTAKE,
        ONLY_FRONT_INTAKE,
        ONLY_BACK_INTAKE,
        REVERSE,
        SHOOTING
    }

    private boolean FrontExtended = false;
    private boolean BackExtended = false;

    private IntakeState m_currentState = IntakeState.HOME;

    // public void updateState(){
    //     if(FrontExtended && BackExtended) {
    //         m_currentState = IntakeState.INTAKE;
    //     }
    //     else if (FrontExtended) {
    //         m_currentState = IntakeState.ONLY_FRONT_INTAKE;
    //     }
    //     else if (BackExtended) {
    //         m_currentState = IntakeState.ONLY_BACK_INTAKE;
    //     }
    //     else {
    //         m_currentState = IntakeState.HOME;
    //     }
    // }

    public void setState(IntakeState p_State) {
        m_currentState = p_State;
    }
    
    public IntakeState getState() {
        return m_currentState;
    }
    
    public boolean isState(IntakeState p_State) {
        return (m_currentState == p_State);
    }

    // public boolean isState(String p_State) {
    //     return (m_currentState.toString().equals(p_State));
    // }

    public IntakeState getCurrentState(){
        return m_currentState;
    }
    
}