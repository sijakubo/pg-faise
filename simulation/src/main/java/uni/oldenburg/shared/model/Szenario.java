package uni.oldenburg.shared.model;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

import uni.oldenburg.client.view.MainFrameView;

/**
 * szenario class to store and handle related information
 * 
 * @author Matthias
 */

@SuppressWarnings("serial")
public class Szenario implements Serializable {
	public static final String TABLE_NAME = "szenario";
	
	private int id;
	private String title;
	private String time_created;
	private String created_by_user;
	private List<Conveyor> lstConveyor = new ArrayList<Conveyor>();
	
	private int width = MainFrameView.canvasWidth;
	private int height = MainFrameView.canvasHeight;	
	
	public Szenario () {
		this.title = "new Szenario";
		this.time_created = "now";
		this.created_by_user = "unknown user";
	}

	public Szenario (String title, String time_created, String created_by_user) {
		this.title = title;
		this.time_created = time_created;
		this.created_by_user = created_by_user;
	}
	
	public void addConveyor(Conveyor newConveyor) {
		lstConveyor.add(newConveyor);
	}
	
	public void removeConveyor(Conveyor conveyor) {
		lstConveyor.remove(conveyor);
	}
	
	public Conveyor getConveyorById(int id){
		for(int i=0;i<lstConveyor.size();i++){
			Conveyor c=lstConveyor.get(i);
			if(c.getID()==id){
				return c;
			}
		}
			return null;//no match found
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
	
	public int getWidth() {
		return width;
	}

	public int getHeight() {
		return height;
	}

	public int getId() {
		return id;
	}

	public void setID(int id) {
		this.id = id;
	}
}
