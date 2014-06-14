package uni.oldenburg.shared.model.event;

import de.novanic.eventservice.client.event.Event;

/**
 * @author Christopher
 */

@SuppressWarnings("serial")
public class JobCounterUpdateEvent implements Event {
	int conveyorID = 0;
	int jobCounter = 0;
	
	public JobCounterUpdateEvent () {}
	
	public JobCounterUpdateEvent (int conveyorID, int jobCounter) {
		this.conveyorID = conveyorID;
		this.jobCounter = jobCounter;
	}
	
	public int getJobCounter() {
		return jobCounter;
	}
	
	public int getConveyorID() {
		return conveyorID;
	}
}