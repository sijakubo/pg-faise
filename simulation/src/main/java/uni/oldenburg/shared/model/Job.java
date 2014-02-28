package uni.oldenburg.shared.model;

public class Job {
    private final int jobNumber;
    private final String job;

    public Job(int jobNumber, String job) {
    	this.jobNumber = jobNumber;
      	this.job = job;
    }

    public int getJobNumber() {
    	return jobNumber;
    }
    
    public String getJob() {
    	return job;
    }
}