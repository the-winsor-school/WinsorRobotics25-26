/* AutonStrategies/IState.java */
package org.firstinspires.ftc.teamcode.AutonStrategies;

public interface IState {
    /**
     * Execute this state and return the next state to execute
     * @return the next IState, or null to end the state machine
     */
    IState execute();
}
