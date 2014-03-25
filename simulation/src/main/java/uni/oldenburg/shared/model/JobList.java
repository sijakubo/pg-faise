package uni.oldenburg.shared.model;

import java.util.ArrayList;
import java.util.List;

public class JobList {
	
	private String name;
	private List<Job> jobList;
	
	public JobList(String name) {
		this.name = name;
		jobList = new ArrayList<Job>();
	}
	
	public void addJob(Job job) {
		int index = 0;
		//sort by timestamp
		while(jobList.get(index) != null && jobList.get(index).getTimestamp() < job.getTimestamp()) {
			index++;
		}
		jobList.add(index, job);
	}
	
	public void addRandomJobs(int numberOfJobs) {
		for(int i = 0; i < numberOfJobs; i++) {
			
			String jobName = "Job #" + i;
			int type = (Math.random() < 0.5) ? Job.INCOMING : Job.OUTGOING;
			long timestamp = (long)(Math.random() * 10000);
			int destinationId = (int)(Math.random() * 1000);
			int packageId = (int)(Math.random() * 1000);

			addJob(new Job(jobName, type, timestamp, destinationId, packageId));
		}
	}
	
	public String getName() {
		return name;
	}
	
	public Job getJob(int index) {
		return jobList.get(index);
	}
	
	public int size() {
		return jobList.size();
	}
}
