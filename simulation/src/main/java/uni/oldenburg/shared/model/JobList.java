package uni.oldenburg.shared.model;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * @author Christopher
 */

@SuppressWarnings("serial")
public class JobList implements Serializable {
	public static final String TABLE_NAME = "joblist";
	private String name;
	private List<Job> jobList;
	
	public JobList() {
		this.name = "undefined";
		this.jobList = new ArrayList<Job>();		
	}
	
	public JobList(String name) {
		this.name = name;
		this.jobList = new ArrayList<Job>();
	}
	
	public void addJob(Job job) {
			
		int index = 0;
		
		//sort by timestamp
		while(index < jobList.size() && jobList.get(index).getTimestamp() < job.getTimestamp()) {
			index++;
		}
		
		if(index == jobList.size()) {
			jobList.add(job);
		} else {
			jobList.add(index, job);
		}
	}
	
	public void addRandomJobs(int numberOfJobs) {
		for(int i = 0; i < numberOfJobs; i++) {
			
			int type = (Math.random() < 0.5) ? Job.INCOMING : Job.OUTGOING;
			int destinationId = (int)(Math.random() * 1000);
			int packageId = (int)(Math.random() * 1000);
						
			int expectedValue = 500;
			int standardDeviation = 100;
			
			int timestamp =  (int) ((expectedValue + (standardDeviation * new Random().nextGaussian())) % 1000);
			
			addJob(new Job(type, timestamp, destinationId, packageId));
		}
	}
	
	public String getName() {
		return name;
	}
	
	public void setName(String name) {
		this.name=name;
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
	
	public List<Job> getJoblist() {
		return this.jobList;
	}
}
