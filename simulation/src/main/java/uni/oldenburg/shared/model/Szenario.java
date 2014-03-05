package uni.oldenburg.shared.model;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("serial")
public class Szenario  implements Serializable {
	private String title;
	private String time_created;
	private String created_by_user;
	private List<Conveyor> lstConveyor;
	
	public Szenario (String title, String time_created, String created_by_user) {
		this.title = title;
		this.time_created = time_created;
		this.created_by_user = created_by_user;
		
		lstConveyor = new ArrayList<Conveyor>();
	}
	
	public void addConveyor(Conveyor newConveyor) {
		lstConveyor.add(newConveyor);
	}
	
	public List<Conveyor> getConveyorList() {
		return lstConveyor;
	}
	
	public String getTitle() {
		return title;
	}

	public String getTimeCreated() {
		return time_created;
	}

	public String getCreatedByUser() {
		return created_by_user;
	}
}
