package org.firstinspires.ftc.teamcode.AutonStrategies;

/**
 * IState is an interface for State Machine states.
 * Each state executes once and returns the next state to execute.
 */
public interface IState
{
    /**
     * Execute this state once.
     * @return the next IState to execute, or null to end the state machine
     */
    IState execute();
}
