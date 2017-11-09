import java.io.*; 
import java.net.*; 
/**
 * Maximum LEGO EV3: Building Robots with Java Brains
 * ISBN-13: 9780986832291
 * Variant Press (C) 2014
 * Chapter 14 - Client-Server Robotics
 * Robot: EV3 Brick * Platform: LEGO EV3
 * @author Brian Bagnall
 * @version July 20, 2014
 */ 
 
public class PCClient { 
	public static void main(String[] args) throws IOException { 
		String ip = "10.0.1.1"; 
		// BT 
		if(args.length > 0) 
			ip = args[0]; 
			
		Socket sock = new Socket(ip, 1234); 
		System.out.println("Connected"); 
		
		InputStream in = sock.getInputStream(); 
		DataInputStream dIn = new DataInputStream(in); 

		while(dIn.available() == 0){
			// Wait
		}
		
		String str = "";
		String lastMessage = "\n";
		
		while(!str.equals("Finished")){
			try {
				str = dIn.readUTF();
			} catch(Exception e){
				
			}
			
			if(!str.equals(lastMessage))
				System.out.println(str);
			
		}

		sock.close(); 
	}
}