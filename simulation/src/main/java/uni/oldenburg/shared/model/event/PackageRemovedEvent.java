package uni.oldenburg.shared.model.event;

import de.novanic.eventservice.client.event.Event;

public class PackageRemovedEvent implements Event {
    private int id;
	
	public PackageRemovedEvent(){
		
	}
	
    public PackageRemovedEvent(int id){
		this.id=id;
	}

	public int getId() {
		return id;
	}

	public void setId(int id) {
		this.id = id;
	}
}
