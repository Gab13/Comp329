import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.GraphicsLCD;

public class PilotMonitor extends Thread {

	private int delay;
	public PilotRobot robot;
	private String msg;
	
    GraphicsLCD lcd = LocalEV3.get().getGraphicsLCD();
	
    // Constructor
    public PilotMonitor(PilotRobot robot, int delay){
    	this.setDaemon(true);
    	this.delay = delay;
    	this.robot = robot;
    	this.msg = "";
    }
}