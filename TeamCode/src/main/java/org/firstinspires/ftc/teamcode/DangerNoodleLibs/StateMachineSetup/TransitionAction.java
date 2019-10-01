package org.firstinspires.ftc.teamcode.DangerNoodleLibs.StateMachineSetup;
/*
 *  Written by Iron Reign Robotics
 *  Edited by Viperbots Recoil
 */
public interface TransitionAction {
    /**
     *   serves as a state transition action if an action is required; housed in interface to be
     *   implemented by lambda expression (cutstomizable for each stage)
     */
    void transition();
}
