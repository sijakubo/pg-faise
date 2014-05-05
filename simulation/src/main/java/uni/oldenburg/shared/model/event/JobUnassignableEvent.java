package uni.oldenburg.shared.model.event;

import uni.oldenburg.shared.model.Job;
import de.novanic.eventservice.client.event.Event;

/**
 * No Ramp had free capacity to take the job
 * 
 * @author Matthias
 */
@SuppressWarnings("serial")
public class JobUnassignableEvent implements Event {
	private Job myJob = null;
	
	public JobUnassignableEvent(Job newJob) {
		myJob = newJob;
	}
	
	public Job getJob() {
		return myJob;
	}
}