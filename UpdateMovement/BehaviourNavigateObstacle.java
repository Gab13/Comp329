/*
 *  COMP329 Assignment 1
 *  Obstacle navigation behaviour
 *  Activates if robot encounters obstacle
 */

import lejos.robotics.subsumption.Behavior;

public class BehaviourNavigateObstacle implements Behavior {
	public boolean suppressed;
	private PilotRobot pilotRobot;

	// Constructor
	public BehaviourNavigateObstacle(PilotRobot robot){
    	 this.pilotRobot = robot;
    }

	// Finish the action
	public void suppress(){
		suppressed = true;
	}

	// Check if the next grid tile has an obstacle in
	public boolean takeControl(){
		if(!Assignment.canGoForward())
			return true;
		else
			return false;
	}

	// Perform action to navigate obstacle
	public void action() {
			
		pilotRobot.getPilot().stop();
		
		pilotRobot.getPilot().forward();

		while (pilotRobot.getUltrasonicSensor() > 0.05) {		
		}
		pilotRobot.getPilot().backward();
		while (pilotRobot.getUltrasonicSensor() < 0.05) {		
		}
		
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
		
		Assignment.Align(rotate);
		
		while(pilotRobot.getPilot().isMoving() && !suppressed)
			Thread.yield();
		
		
	}
}