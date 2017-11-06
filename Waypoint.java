import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import lejos.robotics.geometry.Point;
import lejos.robotics.navigation.Pose;
 
public class Waypoint extends Point {
	
	protected float maxPositionError = -1;
	
	public Waypoint(double x, double y) {
		super((float)x,(float)y);
	}


	public Waypoint(Point p) {
		super((float) p.getX(),(float) p.getY());
	}
	
	public Waypoint(Pose p) {
		super(p.getX(),p.getY());
	}
	
	public double getMaxPositionError() {
		return maxPositionError;
	}
	
	public void setMaxPositionError(double distance) {
		maxPositionError = (float)distance;
	}
	
	//Return a pose that represents the waypoint 
	public Pose getPose() {
		return new Pose(x,y, maxPositionError);
	}
	
	// Check if the given pose satisfies the condition for this waypoint
	public boolean checkValidity(Pose p) {
		if (maxPositionError >= 0 && 
		    p.distanceTo(this) > maxPositionError) return false;

		return true;
	}

	public void dumpObject(DataOutputStream dos) throws IOException {
		dos.writeFloat(x);
		dos.writeFloat(y);
		dos.flush();
		
	}

	public void loadObject(DataInputStream dis) throws IOException {
	 	x = dis.readFloat();
		y = dis.readFloat();
		
		
	}
	
	
	
	

}