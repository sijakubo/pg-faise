package uni.oldenburg.shared.model.event;

import de.novanic.eventservice.client.event.Event;

@SuppressWarnings("serial")
public class PackageAddedEvent implements Event {
	
	private int id;
	
	public PackageAddedEvent(){
		
	}
	
    public PackageAddedEvent(int id){
		this.id=id;
	}

	public int getId() {
		return id;
	}

	public void setId(int id) {
		this.id = id;
	}
	
	

}
