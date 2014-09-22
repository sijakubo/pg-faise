package uni.oldenburg.shared.model;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * @author Christopher Matthias
 */

@SuppressWarnings("serial")
public class JobList implements Serializable {
	public static final String TABLE_NAME = "joblist";
	public static int lastTimestamp = 0;

	private String name = "undefined";
	private List<Job> lstJobs = new ArrayList<Job>();
	private List<Integer> lstOutgoingPackageIDs = new ArrayList<Integer>();

	public JobList() {
		lastTimestamp=0;
		Job.idCounterPacket=0;
	}

	public JobList(String name) {
		this.name = name;
	}

	public void addJob(Job job) {
		lstJobs.add(job);
		Collections.sort(lstJobs);
	}
	
	public void addRandomJobs(int numberOfJobs) {
		for (int i = 0; i < numberOfJobs; i++) {
			int destinationId = (Math.random() < 0.5) ? 0 : -1;

			//int expectedValue = 6;
			//int standardDeviation = 3;

			int timestamp = (lastTimestamp) + 1; // + (int) ((expectedValue + (standardDeviation * new Random().nextGaussian())) % 1000);
			lastTimestamp = timestamp;
			
			addJob(new Job(timestamp, destinationId, this));
		}
	}
	
	public boolean isPackageIDAvailableOutgoing(int packageId) {
		return !lstOutgoingPackageIDs.contains(packageId);
	}
	
	public void registerOutgoingPacketID(int packageId) {
		// only add once
		if (!lstOutgoingPackageIDs.contains(packageId))
			lstOutgoingPackageIDs.add(packageId);
	}

	public String getName() {
		return name;
	}

	public void setName(String name) {
		this.name = name;
	}

	public Job getJob(int index) {
		return lstJobs.get(index);
	}

	public int size() {
		return lstJobs.size();
	}

	public List<Job> subList(int start, int end) {
		return lstJobs.subList(start, end);
	}

	public void removeJob(int packageID, int type) {
		for (Job myJob : lstJobs) {
			if (myJob.getPackageId() == packageID && myJob.getType() == type)
				lstJobs.remove(myJob);
		}
	}
	
	public void removeJob(Job job) {
		if (job.getType() == Job.INCOMING)
			this.registerOutgoingPacketID(job.getPackageId());
		
		lstJobs.remove(job);
	}

	public List<Job> getJoblist() {
		return this.lstJobs;

	}
}
