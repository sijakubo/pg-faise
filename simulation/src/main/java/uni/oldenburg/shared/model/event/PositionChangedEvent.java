package uni.oldenburg.shared.model.event;

import de.novanic.eventservice.client.event.Event;

@SuppressWarnings("serial")
public class PositionChangedEvent implements Event {
	private int id;
	private int x;
	private int y;

	public PositionChangedEvent() {}

	public PositionChangedEvent(int x, int y, int id) {
         this.x=x;
         this.y=y;
         this.id=id;
	}

	public int getId() {
		return id;
	}

	public void setId(int id) {
		this.id = id;
	}

	public int getX() {
		return x;
	}

	public void setX(int x) {
		this.x = x;
	}

	public int getY() {
		return y;
	}

	public void setY(int y) {
		this.y = y;
	}

}
