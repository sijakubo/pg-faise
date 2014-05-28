package uni.oldenburg.shared.model.event;

import de.novanic.eventservice.client.event.Event;

@SuppressWarnings("serial")
public class BotRemovePackageEvent implements Event {
	private int id;

	public BotRemovePackageEvent() {

	}

	public BotRemovePackageEvent(int id) {
		this.id = id;
	}

	public int getId() {
		return id;
	}

	public void setId(int id) {
		this.id = id;
	}
}
