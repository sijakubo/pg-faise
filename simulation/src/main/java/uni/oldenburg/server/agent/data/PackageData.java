package uni.oldenburg.server.agent.data;

import uni.oldenburg.shared.model.Job;
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
	int type = -1; // IMCOMING = 0 | OUTGOING = 1
	

	
	public PackageData() {};
	
	public PackageData(int packageID, int destinationID) {
		this.setPackageID(packageID);
		this.setDestinationID(destinationID);
		
		this.type = destinationID == 0 ? Job.INCOMING : Job.OUTGOING;		
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
	

	public int getType() {
		return this.type;
	}
	
	

}
