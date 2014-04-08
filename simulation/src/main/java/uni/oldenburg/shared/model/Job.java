package uni.oldenburg.shared.model;

import java.io.Serializable;

/**
 * @author Christopher Matthias
 */

@SuppressWarnings("serial")
public class Job implements Serializable {
	public static final String TABLE_NAME = "job";
	
	public static final int INCOMING = 0;
	public static final int OUTGOING = 1;
	
	private static int idCounterPacket = 0;	
	
    private int type = -1;
    private int timestamp = 0;
    private int destinationId = 0;
    private int packageId = 0;
    
    public Job(int timestamp, int destinationId) {
      	this.timestamp = timestamp;
      	this.destinationId = destinationId;    	
      	this.type = destinationId == 0 ? Job.INCOMING : Job.OUTGOING;
      	
		if (type == Job.INCOMING)
			this.packageId = ++idCounterPacket;
		else {
			this.packageId = (int) ((Math.random() * 10000) % idCounterPacket);

			if (Math.random() < 0.1 || idCounterPacket < 5)
				packageId += 5 + (packageId / 10);
		}      	
    }
    
    public Job() {}
    
    public int getDestinationId() {
    	return destinationId;
    }
    
    public int getPackageId() {
    	return packageId;
    }
    
    public int getType() {
    	return type;
    }
    
    public int getTimestamp() {
    	return timestamp;
    }
}