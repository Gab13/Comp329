/*
 *  COMP329 Assignment 1
 *  Navigate map behaviour
 *  Navigates the map to generate occupancy grid + visual map
 */

import lejos.robotics.subsumption.Behavior;
import lejos.robotics.mapping.NavigationModel;
import lejos.hardware.Button;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;

public class BehaviourNavigateMap implements Behavior {
	public boolean suppressed;
	private PilotRobot pilotRobot;
	private int travelLength;
	private int travel;

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
		//G.R//
		// Allow this method to run
		suppressed = false;
		
		pilotRobot.getPilot().stop();
		
		//If the blue colour is detected
		if (pilotRobot.getColour() == 2) {
			System.out.println("Blue detected");
			System.out.println(Assignment.getPose());
		}

		Assignment.scanWithSensor();
		pilotRobot.getUltrasonicMotor().rotate(90);
		Assignment.scanWithSensor();
		pilotRobot.getUltrasonicMotor().rotate(-180);
		Assignment.scanWithSensor();
		pilotRobot.getUltrasonicMotor().rotateTo(0);

		pilotRobot.getPilot().stop();
		
		travel = 30;
		double head = Assignment.getPose().getHeading();
		if ((head > 80) && (head < 100))
			travel = 30;
		if ((head < -80) && (head > -100))
			travel = 30;
		else
			travel = 35;
		
		pilotRobot.getPilot().travel(30);
		
		//G.R//

		while(!suppressed && pilotRobot.getPilot().isMoving()) {
			Thread.yield();
		}
	}
}