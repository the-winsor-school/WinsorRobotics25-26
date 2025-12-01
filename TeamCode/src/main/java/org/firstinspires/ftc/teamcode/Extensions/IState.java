package org.firstinspires.ftc.teamcode.Extensions;

public interface IState {
    public void enter();
    public void execute();
    public void exit();
    public boolean shouldTransition();
}
