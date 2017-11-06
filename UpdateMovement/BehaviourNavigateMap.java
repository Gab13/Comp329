/*
 *  COMP329 Assignment 1
 *  Navigate map behaviour
 *  Navigates the map to generate occupancy grid + visual map
 */

import lejos.robotics.subsumption.Behavior;

public class BehaviourNavigateMap implements Behavior {
	public boolean suppressed;
	private PilotRobot pilotRobot;
	private int travelLength;

	// Constructor
	public BehaviourNavigateMap(PilotRobot robot){
    	 this.pilotRobot = robot;
    }

	// Finish the action
	public void suppress(){
		suppressed = true;
	}

	// Check if the map has been completed or not
	public boolean takeControl(){
		if(!Assignment.isMapFinished()) {
			//Assignment.printMessage("Moving...");
			return true;
		} else {
			//Assignment.printMessage("Map is finished");
			return false;
		}
	}

	// Perform action to navigate map
	public void action() {
		// Allow this method to run
		suppressed = false;
		
		pilotRobot.getPilot().stop();
		
		Assignment.scanWithSensor();

		Assignment.canGoForward();
		
		pilotRobot.getPilot().travel(30);

		while(!suppressed && pilotRobot.getPilot().isMoving()) {
			Thread.yield();
		}
	}
}