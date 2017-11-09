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
		//if(Assignment.isObstacleInNextTile())
			return true;
		else
			return false;
	}


	public void action() {
		//G.R//

		// Allow this method to run
		suppressed = false;
		pilotRobot.getPilot().stop();

		pilotRobot.getPilot().forward();

		while (pilotRobot.getUltrasonicSensor() > 0.05) {		

		}

		pilotRobot.getPilot().backward();

		while (pilotRobot.getUltrasonicSensor() < 0.05) {		

		}

		pilotRobot.getPilot().stop();

		// Default to turn around
		int rotate = 180;

		// Can we rotate right?
		if(Assignment.canRotateRight())
			rotate = -90;

		// Can we rotate left?
		else if(Assignment.canRotateLeft())
			rotate = 90;

		pilotRobot.getPilot().rotate(rotate);

		//Assignment.Align(rotate);
		
		int[] currentCell = Assignment.getEstimatedLocation();
		
		int h = currentCell[2];
		
		if(h >= 45 && h < 135) {
			// Facing right from original position
			if(rotate == 90)
				h = 180;
			else if(rotate == -90)
				h = 0;
			else
				h = 270;
			
		} else if(h >= 135 && h < 225) {
			// Facing back towards original position
			if(rotate == 90)
				h = 270;
			else if(rotate == -90)
				h = 90;
			else
				h = 0;
			
		} else if(h >= 225 && h < 315) {
			// Facing left
			if(rotate == 90)
				h = 0;
			else if(rotate == -90)
				h = 180;
			else
				h = 90;
			
		} else {
			// Facing forwards
			if(rotate == 90)
				h = 90;
			else if(rotate == -90)
				h = 270;
			else
				h = 180;
			
		}
		
		Assignment.setLocation(currentCell[0], currentCell[1], h);

		//G.R//
		while(pilotRobot.getPilot().isMoving() && !suppressed)
			Thread.yield();

	}
}