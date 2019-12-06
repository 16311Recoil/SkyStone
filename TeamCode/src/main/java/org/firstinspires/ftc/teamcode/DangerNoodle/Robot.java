package org.firstinspires.ftc.teamcode.DangerNoodle;

public interface Robot {
    void scan();
    /*
     scan() method scans the lined up stones looking for the skystones:
     1. Scans the Skystone blocks during init()
     2. Determines position of the block and stores it for use in the retrieval method.
     */
    void retrieval(double distance);
    /*
     retrieval() method:
      1. Drives to the Skystone
      2. Collects the Skystone
      3. Drives to the foundation.
      4. Releases Skystone onto the foundation.
     */
    void moveFoundation(boolean blue, boolean skybridge) throws InterruptedException;
    /*
    moveFoundation() method takes a direction, whether it is moving the foundation towards the wall
    or away from the field wall and latches onto the foundation (assuming the robot is in correct
    position) and moves the foundation in said direction.
     */
    void navigate(boolean inside);
    /*
    navigate() method parks the robot in the precise location necessary during auto, on the inside
    lane or the outside lane, depending on the necessity.
     */
    void placeSkystone();
    /*
    placeSkystone() uses the necessary manipulator class in order to place the Skystone according to
    competition goals.
    - Can drop the skystone or can place the skystone
     */

}
