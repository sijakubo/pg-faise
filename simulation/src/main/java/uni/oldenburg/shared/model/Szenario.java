package uni.oldenburg.shared.model;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 * szenario class to store and handle related information
 * 
 * @author Matthias
 */

@SuppressWarnings("serial")
public class Szenario implements Serializable {
	public static final String TABLE_NAME = "szenario";
	
	private String title;
	private String time_created;
	private String created_by_user;
	private List<Conveyor> lstConveyor;
	
	public Szenario () {
		this.title = "new Szenario";
		this.time_created = "now";
		this.created_by_user = "unknown user";
		
		lstConveyor = new ArrayList<Conveyor>();
	};
		
	public Szenario (String title, String time_created, String created_by_user) {
		this.title = title;
		this.time_created = time_created;
		this.created_by_user = created_by_user;
		
		lstConveyor = new ArrayList<Conveyor>();
	}
	
	public void addConveyor(Conveyor newConveyor) {
		lstConveyor.add(newConveyor);
	}
	
	public void removeConveyor(Conveyor conveyor) {
		lstConveyor.remove(conveyor);
	}
	
	public String getTitle() {
		return title;
	}

	public void setTitle(String title) {
		this.title = title;
	}

	public String getTimeCreated() {
		return time_created;
	}

	public void setTimeCreated(String time_created) {
		this.time_created = time_created;
	}

	public String getCreatedByUser() {
		return created_by_user;
	}

	public void setCreatedByUser(String created_by_user) {
		this.created_by_user = created_by_user;
	}

	public List<Conveyor> getConveyorList() {
		return lstConveyor;
	}
}
