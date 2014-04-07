package uni.oldenburg.shared.model;

import java.io.Serializable;

/**
 * @author Christopher Matthias
 */




public class Job implements Serializable {
	
	public static final String TABLE_NAME = "job";

	public static final int INCOMING = 0;
	public static final int OUTGOING = 1;
	
	private static int idCounterJob = 0;
	
    private int id;
    private int type;
    private int timestamp;
    private int destinationId;
    private int packageId;
    
    public Job(int id, int type, int timestamp, int packageId, int destinationId) {
    	this.id = id;
      	this.type = type;
      	this.timestamp = timestamp;
      	this.packageId = packageId;
      	this.destinationId = destinationId;
    }
    

   
    
    public Job(int type, int timestamp, int packageId, int destinationId) {
	    this.id = ++idCounterJob;
      	this.type = type;
      	this.timestamp = timestamp;
      	this.packageId = packageId;
      	this.destinationId = destinationId;
    }
    
    public Job(int timestamp, int packageId, int destinationId) {
    	this.id = ++idCounterJob;
      	this.type = -1;
      	this.timestamp = timestamp;
      	this.packageId = packageId;
      	this.destinationId = destinationId;
    }
    
    public Job( ) {
    	this.id = ++idCounterJob;
      	this.type = -1;
      	this.timestamp = 0;
      	this.packageId = 0;
      	this.destinationId = 0;
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