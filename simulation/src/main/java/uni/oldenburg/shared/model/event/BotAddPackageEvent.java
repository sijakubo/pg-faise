package uni.oldenburg.shared.model.event;

import de.novanic.eventservice.client.event.Event;

public class BotAddPackageEvent implements Event{
     private int id;
	
	public BotAddPackageEvent(){
		
	}
	
    public BotAddPackageEvent(int id){
		this.id=id;
	}

	public int getId() {
		return id;
	}

	public void setId(int id) {
		this.id = id;
	}
}
