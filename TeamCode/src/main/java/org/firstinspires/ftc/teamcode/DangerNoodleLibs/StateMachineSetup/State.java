package org.firstinspires.ftc.teamcode.DangerNoodleLibs.StateMachineSetup;

public class State {
    // Conveys the "State" of the state machine.
    private int stage;

    // Default Constructor - inits stage to first stage.
    public State(){
        stage = 0;
    }

    // Parameter constructor - inits stage to input stage
    // could be used to skip stages based on StateMachine inputs.

    /** @param stage - inits to desired stage*/
    public State(int stage){
        this.stage = stage;
    }

    // returns stage
    public int getStage() {
        return stage;
    }

    /** @param stage - desired stage to set to*/
    public void setStage(int stage){
        this.stage = stage;
    }

    // increments stage (moving the stage from one state to another).
    public void incrementStage(){
        stage++;
    }

    // Prints stage
    @Override
    public String toString(){
        return Integer.toString(this.stage);
    }
}
