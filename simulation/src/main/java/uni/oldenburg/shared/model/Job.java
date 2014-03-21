package uni.oldenburg.shared.model;

public class Job {
	
	public static final int INCOMING = 0;
	public static final int OUTGOING = 1;
	
    private final int jobNumber;
    private final String job;
    private final int type;
    private final long timestamp;

    public Job(int jobNumber, String job, int type, long timestamp) {
    	this.jobNumber = jobNumber;
      	this.job = job;
      	this.type = type;
      	this.timestamp = timestamp;
    }

    public int getJobNumber() {
    	return jobNumber;
    }
    
    public String getJob() {
    	return job;
    }
    
    public int getType() {
    	return type;
    }
    
    public long getTimestamp() {
    	return timestamp;
    }
}