package uni.oldenburg.server.agent.data;

import jade.util.leap.Serializable;

/**
 * to save relevant package informations
 * 
 * @author Matthias
 */	
@SuppressWarnings("serial")
public class PackageData implements Serializable {
	int packageID = 0;
	int destinationID = 0;
	boolean reserved=false;
	
	public PackageData() {};
	
	public PackageData(int packageID, int destinationID) {
		this.setPackageID(packageID);
		this.setDestinationID(destinationID);
	}
	
	public int getPackageID() {
		return packageID;
	}
	
	public void setPackageID(int packageID) {
		this.packageID = packageID;
	}
	
	public int getDestinationID() {
		return destinationID;
	}
	
	public void setDestinationID(int destinationID) {
		this.destinationID = destinationID;
	}
	
	public boolean isReserved(){
		return this.reserved;
	}
	
	public void setReserved(){
		this.reserved=true;
	}
	
	public void setUnReserved(){
		this.reserved=false;
	}
	
}
