package uni.oldenburg.shared.model;

import java.io.Serializable;

/**
 * @author Christopher
 */
@SuppressWarnings("serial")
public class Job implements Serializable {
	public static final String TABLE_NAME = "job";
	public static final int INCOMING = 0;
	public static final int OUTGOING = 1;
	
	private static int IdCounter = 0;
	
    private final int id;
    private final int type;
    private final int timestamp;
    private final int destinationId;
    private final int packageId;
    
    public Job(int type, int timestamp, int packageId, int destinationId) {
    	this.id = ++IdCounter;
      	this.type = type;
      	this.timestamp = timestamp;
      	this.packageId = packageId;
      	this.destinationId = destinationId;
    }
    
    public Job( int timestamp, int packageId, int destinationId) {
    	this.id = ++IdCounter;
      	this.type = -1;
      	this.timestamp = timestamp;
      	this.packageId = packageId;
      	this.destinationId = destinationId;
    }

    public int getDestinationId() {
    	return destinationId;
    }

    public int getId() {
    	return id;
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