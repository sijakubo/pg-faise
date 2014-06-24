package uni.oldenburg.shared.model.event;

import de.novanic.eventservice.client.event.Event;

/**
 * @author Christopher
 */

@SuppressWarnings("serial")
public class JobCounterUpdateEvent implements Event {
	private int conveyorID = 0;
	private boolean jobAdded = true; // if false, a job is being removed
	
	@Deprecated
	public JobCounterUpdateEvent () {}
	
	public JobCounterUpdateEvent (int conveyorID, boolean added) {
		this.conveyorID = conveyorID;
		this.jobAdded = added;
	}
	
	public int getConveyorID() {
		return conveyorID;
	}
	
	public boolean isJobAdded() {
		return jobAdded;
	}
}