package org.firstinspires.ftc.teamcode.DangerNoodleLibs.StateMachineSetup;

import java.util.ArrayList;
import java.util.List;
import java.util.TreeMap;

public class StateMachineBuilder {
    private EndAction endAction;
    private TransitionAction transitionAction;
    private ArrayList<State> stateOrder;

    // Constructor constructs a new builder
    public StateMachineBuilder(){
        stateOrder = new ArrayList<State>();

        // Must be implemented by lambda expression;
        endAction = null;
        transitionAction = null;
    }

    // Constructs a builder based on given order of states, given endAction method, and given transition action method.
    public StateMachineBuilder(ArrayList<State> states, final EndAction endAction, final TransitionAction transitionAction){
        this.stateOrder = states;
        this.endAction = endAction;
        this.transitionAction = transitionAction;
    }

    // returns a builder object based on specifications.
    public StateMachineBuilder getBuilder(ArrayList<State> states, final EndAction endAction, final TransitionAction transitionAction){
        this.stateOrder = states;
        this.endAction = endAction;
        this.transitionAction = transitionAction;
        return this;
    }

    // Checks to ensure that no instance variables are invalid in order to initialize a state machine.
    public void build() throws NullStateException {
        if (this.transitionAction == null)
            throw new NullStateException("TRANSITION STATE CANNOT BE NULL");
        if (this.endAction == null)
            throw new NullStateException("END ACTION CANNOT BE NULL");
        if (stateOrder.size() == 0)
            throw new NullStateException("NO STATES INITIALIZED");
    }

}
