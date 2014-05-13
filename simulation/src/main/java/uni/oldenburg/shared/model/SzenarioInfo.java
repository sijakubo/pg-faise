package uni.oldenburg.shared.model;

import java.io.Serializable;

@SuppressWarnings("serial")
public class SzenarioInfo implements Serializable {
	public String Title = "";
	public int EntryRampCount = 0;
	public int ExitRampCount = 0;
	public int StorageRampCount = 0;
	public int VehicleCount = 0;
	
	public SzenarioInfo() {}
}
