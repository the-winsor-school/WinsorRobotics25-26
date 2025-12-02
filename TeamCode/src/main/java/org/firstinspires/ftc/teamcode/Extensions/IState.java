package org.firstinspires.ftc.teamcode.Extensions;

/**
 * Here's the State interface
 * We don't need it in the AutonStrategy thing.
 */
public interface IState {

    /**
     * This is ONE step in the state.
     * The method should return the next state (potentially itself)
     * @return
     */
    public IState execute();
}
