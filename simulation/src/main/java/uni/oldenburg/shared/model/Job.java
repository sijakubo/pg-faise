package uni.oldenburg.shared.model;

import java.io.Serializable;

/**
 * @author Christopher Matthias
 */

@SuppressWarnings("serial")
public class Job implements Serializable, Comparable<Job> {
	public static final String TABLE_NAME = "job";
	
	public static final int INCOMING = 0;
	public static final int OUTGOING = 1;
	
	private static int idCounterPacket = 0;
	
    private int type = -1;
    private int timestamp = 0;
    private int destinationId = 0;
    private int packageId = 0;
    
    public Job() {}
    
    public Job(int timestamp, int destinationId, JobList jobList) {
      	this.timestamp = timestamp;
      	this.destinationId = destinationId;
      	this.type = destinationId == 0 ? Job.INCOMING : Job.OUTGOING;
      	
		if (type == Job.INCOMING)
			createIncoming();
		else {
			// create incoming if outgoing could not
			// be created after several attempts
			if (createOutgoing(jobList) == false)
				createIncoming();
		}      	
    }
    
    private boolean createOutgoing(JobList jobList) {
    	boolean foundUseablePackageID = false;
    	final int iCounterMax = 10;    	
    	int iCounter = 0;
    	
    	do {
			this.packageId = (int) ((Math.random() * 100000) % idCounterPacket);
			
			if (this.packageId < 1)
				this.packageId = 1;

			if (Math.random() < 0.1 || idCounterPacket < 5)
				packageId += 5 + (packageId / 10);
			
			foundUseablePackageID = jobList.isPackageIDAvailableOutgoing(this.packageId);
		} while(!foundUseablePackageID && (++iCounter <= iCounterMax));
    	
    	// avoid using the same packageId again
    	if (foundUseablePackageID)
    		jobList.registerOutgoingPacketID(this.packageId);
    	
    	return foundUseablePackageID;
    }
    
    private void createIncoming() {
    	this.destinationId = 0;
    	this.type = Job.INCOMING;
    	this.packageId = ++idCounterPacket;
    }
    
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

    public int compareTo(Job job) {
		if (this.packageId < job.packageId)
            return -1;
		
        if (this.packageId == job.packageId)
            return 0;
           
        return 1;
	}
}