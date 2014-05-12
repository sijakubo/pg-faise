package uni.oldenburg.shared.model.event;

import uni.oldenburg.shared.model.Job;
import de.novanic.eventservice.client.event.Event;

/**
 * Job was successfully placed on a ramp with available capacity
 * 
 * @author Matthias
 */
@SuppressWarnings("serial")
public class JobAssignedEvent implements Event {
	private Job myJob = null;
	
	
	public JobAssignedEvent() {
		
	}
	
	public JobAssignedEvent(Job newJob) {
		myJob = newJob;
	}
	
	public Job getJob() {
		return myJob;
	}
}