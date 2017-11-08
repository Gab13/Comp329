import lejos.robotics.subsumption.Behavior;

public class BehaviourIsMapFinished implements Behavior {
	public boolean suppressed;
	private PilotRobot pilotRobot;
	private int travelLength;
	private int travel;

	// Constructor
	public BehaviourIsMapFinished(PilotRobot robot){
    	 this.pilotRobot = robot;
    }

	// Finish the action
	public void suppress(){
		suppressed = true;
	}

	// Check if the map has been completed or not
	public boolean takeControl(){
		if(Assignment.canGoToFinalCell) {
			return true;
		} else {
			return false;
		}
	}

	// Go to final cell
	public void action() {
		suppressed = false;
		
		
	}
}
