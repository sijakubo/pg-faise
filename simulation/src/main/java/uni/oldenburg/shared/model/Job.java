package uni.oldenburg.shared.model;

/**
 * @author Christopher Matthias
 */

public class Job {
	public static final int INCOMING = 0;
	public static final int OUTGOING = 1;
	
	private static int idCounterJob = 0;
	
    private final int id;
    private final int type;
    private final long timestamp;
    private final int destinationId;
    private final int packageId;
    
    public Job(int type, long timestamp, int packageId, int destinationId) {
    	this.id = ++idCounterJob;
      	this.type = type;
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
    
    public long getTimestamp() {
    	return timestamp;
    }
}