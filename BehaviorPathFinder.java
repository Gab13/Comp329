import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;
import lejos.robotics.ColorIdentifier;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.Pose;
import lejos.robotics.subsumption.Behavior;
import lejos.robotics.localization.OdometryPoseProvider;



public class BehaviorPathFinder implements Behavior {
	
	public boolean suppressed;
	
	private static OdometryPoseProvider opp;
	
	// Constructor
	public void BehaviourPathFinder(PilotRobot robot){
    }
	
	// Finish the action
	public void suppress(){
		suppressed = true;
	}
	
	// Create sensor
    Port port = LocalEV3.get().getPort("S3");
    EV3ColorSensor sensor = new EV3ColorSensor(port);

    // Use ColorID Mode
    SampleProvider provider = sensor.getMode("ColorID");

    // To store data
    float sample[] = new float[provider.sampleSize()];

	private PilotRobot pilotRobot;
    
    
    @Override
	public boolean takeControl(){
    	
    	opp = new OdometryPoseProvider(pilotRobot.getPilot());
    	Pose pose = opp.getPose();
    	 
		int colorValue = ((ColorIdentifier) pose).getColorID(); 
		Thread.yield();

		if (colorValue != Color.BLUE) 
			return false;
		
		else if (colorValue == Color.BLUE) {
			
			float xCoordinate = pose.getX();
			float yCoordinate = pose.getY();
			
			new Waypoint(xCoordinate, yCoordinate);
			return true;
		}
		return suppressed; 
	}

   
    


	


	@Override
	public void action() {
		// TODO Auto-generated method stub
		
	}


}
