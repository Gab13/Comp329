/*
 *  COMP329 Assignment 1
 *  Bumpers pressed behaviour
 *  Activates if robot's front bumpers are pressed
 */

import lejos.robotics.subsumption.Behavior;

public class BehaviourBumpersPressed implements Behavior {
	public boolean suppressed;
	private PilotRobot pilotRobot;

	// Constructor
	public BehaviourBumpersPressed(PilotRobot robot){
    	 this.pilotRobot = robot;
    }

	// Finish the action
	public void suppress(){
		suppressed = true;
	}

	// Check if the bumpers are pressed
	public boolean takeControl(){
		if(pilotRobot.isLeftBumperPressed() || pilotRobot.isRightBumperPressed()) {
			return true;
		}
		else
			return false;
	}

	// Perform action to navigate obstacle
	public void action() {
		
		pilotRobot.getPilot().stop();
		
		// Allow this method to run
		suppressed = false;
		
		// Default to turn around
		int rotate = 180;
		
		// Can we rotate right?
		if(Assignment.canRotateRight())
			rotate = 90;
		
		// Can we rotate left?
		else if(Assignment.canRotateLeft())
			rotate = -90;
		
		pilotRobot.getPilot().rotate(rotate);
		
		while(pilotRobot.getPilot().isMoving() && !suppressed)
			Thread.yield();
		
	}
}