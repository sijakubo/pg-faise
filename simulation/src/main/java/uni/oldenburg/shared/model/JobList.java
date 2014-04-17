package uni.oldenburg.shared.model;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

/**
 * @author Christopher Matthias
 */

@SuppressWarnings("serial")
public class JobList implements Serializable {
	public static final String TABLE_NAME = "joblist";

	private String name = "undefined";
	private List<Job> jobList = new ArrayList<Job>();
	private List<Integer> lstOutgoingPackageIDs = new ArrayList<Integer>();

	public JobList() {}

	public JobList(String name) {
		this.name = name;
	}

	public void addJob(Job job) {
		jobList.add(job);
		Collections.sort(jobList);
	}
	
	public void addRandomJobs(int numberOfJobs) {
		for (int i = 0; i < numberOfJobs; i++) {
			int destinationId = (Math.random() < 0.5) ? 0 : -1;

			int expectedValue = 500;
			int standardDeviation = 100;

			int timestamp = (int) ((expectedValue + (standardDeviation * new Random().nextGaussian())) % 1000);

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
		return jobList.get(index);

	}

	public int size() {
		return jobList.size();
	}

	public List<Job> subList(int start, int end) {
		return jobList.subList(start, end);
	}

	public void removeJob(Job job) {
		if (job.getType() == Job.INCOMING)
			this.registerOutgoingPacketID(job.getPackageId());
		
		jobList.remove(job);
	}

	public List<Job> getJoblist() {
		return this.jobList;

	}
}
