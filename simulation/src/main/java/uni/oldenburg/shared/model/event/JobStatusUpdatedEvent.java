package uni.oldenburg.shared.model.event;

import de.novanic.eventservice.client.event.Event;

@SuppressWarnings("serial")
public class JobStatusUpdatedEvent implements Event {
	int conveyorID = 0;
	boolean hasPendingJob = false;
	int jobType = 0;
	
	public JobStatusUpdatedEvent () {}
	
	public JobStatusUpdatedEvent(int conveyorID, boolean hasPendingJob, int jobType) {
		this.conveyorID = conveyorID;
		this.hasPendingJob = hasPendingJob;
		this.jobType = jobType;
	}
	
	public boolean getJobStatus() {
		return hasPendingJob;
	}
	
	public int getJobType() {
		return jobType;
	}
	
	public int getConveyorID() {
		return conveyorID;
	}
}