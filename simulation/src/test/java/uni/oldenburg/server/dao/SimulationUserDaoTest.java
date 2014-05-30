package uni.oldenburg.server.dao;

import junit.framework.Assert;

import org.junit.Ignore;
import org.junit.Test;

import uni.oldenburg.shared.model.SimulationUser;

public class SimulationUserDaoTest {
    @Ignore
    @Test
    public void testFindUserForUsernameAndPassword() throws Exception {
		SimulationUserDao simulationUserDao = new SimulationUserDao();
        SimulationUser user = simulationUserDao.findUserForUsernameAndPassword("test@test.de", "blabla");
        Assert.assertNotNull(user);
        Assert.assertEquals(user.getName(), "simon");
        Assert.assertNotNull(user.getId());	
    }
    
    @Ignore
    @Test
    public void testFindUserForID() throws Exception {
		SimulationUserDao simulationUserDao = new SimulationUserDao();
        Long l=new Long(19);
        SimulationUser user=simulationUserDao.findSimulationUserById(l);
        Assert.assertNotNull(user);
        Assert.assertEquals(user.getName(), "nagi");	
    }
    
    @Ignore
    @Test
    public void testPersistUser() throws Exception {
    	//The test should secure, that a persisted user is persisted actually
    	
		SimulationUserDao simulationUserDao = new SimulationUserDao();
    	
    	//create user
    	if(simulationUserDao.findUserForUsernameAndPassword("rasch@inf.de", "nagi")==null){
    		SimulationUser user= new SimulationUser("rasch@inf.de", "rasch", "nagi");
    	
        	//Persist User
        	
        	simulationUserDao.persistSimulationUser(user);
        	
        	//Check if the persisted User was successfully persisted
        	SimulationUser userTest=simulationUserDao.findUserForUsernameAndPassword("rasch@inf.de", "nagi");
        	Assert.assertNotNull(userTest);
        	Assert.assertEquals(userTest.getEmail(),user.getEmail());
        	Assert.assertEquals(userTest.getName(),user.getName());
        	Assert.assertEquals(userTest.getPassword(),user.getPassword());
    	} 	
    }
}

