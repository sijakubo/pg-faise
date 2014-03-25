package uni.oldenburg.shared.model;

public class Job {
	
	public static final int INCOMING = 0;
	public static final int OUTGOING = 1;
	
	private static int IdCounter = 0;
	
    private final int id;
    private final String job;
    private final int type;
    private final long timestamp;
    private final int destinationId;
    private final int packageId;
    
    public Job(String job, int type, long timestamp, int destinationId, int packageId) {
    	this.id = ++IdCounter;
    	this.job = job;
      	this.type = type;
      	this.timestamp = timestamp;
      	this.destinationId = destinationId;
      	this.packageId = packageId;
    }

    public int getDestinationId() {
    	return destinationId;
    }

    public int getId() {
    	return id;
    }
    
    public String getJob() {
    	return job;
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