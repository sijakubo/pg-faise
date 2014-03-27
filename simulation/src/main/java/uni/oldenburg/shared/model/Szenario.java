package uni.oldenburg.shared.model;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("serial")
public class Szenario implements Serializable {
	public static final String TABLE_NAME = "szenario";
	
	private int id;
	private String title;
	private String time_created;
	private String created_by_user;
	private List<Conveyor> lstConveyor;
	private JobList jobList;
	
	public Szenario () {
		this.id = -1;
		this.title = "new Szenario";
		this.time_created = "now";
		this.created_by_user = "unknown user";
		
		lstConveyor = new ArrayList<Conveyor>();
		jobList = new JobList("Auftragsliste");
		jobList.addJob(new Job(Job.OUTGOING, 42, 128, 12)); //dummy-data
		jobList.addJob(new Job(Job.OUTGOING, 73, 256, 45)); //dummy-data
		jobList.addJob(new Job(Job.INCOMING, 512, 128, 0)); //dummy-data
		jobList.addJob(new Job(Job.OUTGOING, 1337, 1024, 128)); //dummy-data
	};
		
	public Szenario (int id, String title, String time_created, String created_by_user) {
		this.id = id;
		this.title = title;
		this.time_created = time_created;
		this.created_by_user = created_by_user;
		
		lstConveyor = new ArrayList<Conveyor>();
		jobList = new JobList("Auftragsliste");
		jobList.addJob(new Job(Job.OUTGOING, 42, 128, 12)); //dummy-data
		jobList.addJob(new Job(Job.OUTGOING, 73, 256, 45)); //dummy-data
		jobList.addJob(new Job(Job.INCOMING, 512, 128, 0)); //dummy-data
		jobList.addJob(new Job(Job.OUTGOING, 1337, 1024, 128)); //dummy-data
	}
	
	public int getID() {
		return this.id;
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
	
	public JobList getJoblist() {
		return jobList;
	}
}
