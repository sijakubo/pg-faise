package uni.oldenburg.server.dao;

import junit.framework.Assert;

import org.junit.Ignore;
import org.junit.Test;

import uni.oldenburg.server.service.RegistrationAndLoginServiceImpl;
import uni.oldenburg.shared.model.SimulationUser;

public class RegistrationAndLoginServletTest {
	
	@Ignore
    @Test
	public void registerUserTest(){
		//Test should secure that the Servlet returns false, if a -user should
		//be registered, which already exists 
		
		SimulationUser user= new SimulationUser("rasch@inf.de", "rasch", "nagi");
		
		RegistrationAndLoginServiceImpl servlet=new RegistrationAndLoginServiceImpl();
		
		Assert.assertFalse(servlet.registerUser(user));
	}
	@Ignore
	@Test
	public void testLoginUserTrue(){
		//Test should secure that the Servlet returns true, if a -user is logged in
		//which is already registered
		
		
		
		RegistrationAndLoginServiceImpl servlet=new RegistrationAndLoginServiceImpl();
		
		Assert.assertTrue(servlet.loginUser("ja","ja"));
	}
	
	@Test
	public void testLoginUserFalse(){
		//Test should secure that the Servlet returns false, if a -user is logged in
		//which is already registered
		
		RegistrationAndLoginServiceImpl servlet=new RegistrationAndLoginServiceImpl();
		
		Assert.assertFalse(servlet.loginUser("hexe","hexe"));
		
		
	}

}
