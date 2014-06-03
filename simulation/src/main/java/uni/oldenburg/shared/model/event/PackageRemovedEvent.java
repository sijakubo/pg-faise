package uni.oldenburg.shared.model.event;

import de.novanic.eventservice.client.event.Event;

@SuppressWarnings("serial")
public class PackageRemovedEvent implements Event {
	private int conveyorID;
	private int packageID;
	
	public PackageRemovedEvent() {}
	
    public PackageRemovedEvent(int conveyorID, int packageID){
		this.setConveyorID(conveyorID);
		this.setPackageID(packageID);
	}

	public int getPackageID() {
		return packageID;
	}

	public void setPackageID(int packageID) {
		this.packageID = packageID;
	}

	public int getConveyorID() {
		return conveyorID;
	}

	public void setConveyorID(int conveyorID) {
		this.conveyorID = conveyorID;
	}
}
