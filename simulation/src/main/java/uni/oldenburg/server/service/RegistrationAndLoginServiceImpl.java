package uni.oldenburg.server.service;

import com.google.gwt.user.server.rpc.RemoteServiceServlet;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.client.service.RegistrationAndLoginService;
import uni.oldenburg.server.dao.SimulationUserDao;
import uni.oldenburg.shared.model.SimulationUser;

import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpSession;

import java.sql.SQLException;

@SuppressWarnings("serial")
public class RegistrationAndLoginServiceImpl extends RemoteServiceServlet implements RegistrationAndLoginService {

    public boolean registerUser(SimulationUser newUser) {
        SimulationUserDao simulationUserDao = new SimulationUserDao();
        boolean persistSuccessful = false;

        try {
            simulationUserDao.persistSimulationUser(newUser);
            persistSuccessful = true;
        } catch (SQLException e) {
            e.printStackTrace();
        }

        return persistSuccessful;
    }


    public boolean loginUser(String email, String password) {
        SimulationUserDao simulationUserDao = new SimulationUserDao();
        SimulationUser user = null;

        try {
            user = simulationUserDao.findUserForUsernameAndPassword(email, password);
        } catch (SQLException e) {
            e.printStackTrace();
        }

        boolean isUserFoundForCredentials = user != null;
        if (isUserFoundForCredentials) {
            storeUserInSession(user);
        } else {
            Logger logger = Logger.getLogger(RegistrationAndLoginService.class);
            logger.log(Level.ERROR, "Unable to find User for email and password: (" + email + "/" + password + ")");
        }

        return isUserFoundForCredentials;
    }

    //Login User
    private void storeUserInSession(SimulationUser user) {
        HttpServletRequest httpServletRequest = this.getThreadLocalRequest();
        HttpSession session = httpServletRequest.getSession(true);
        session.setAttribute("user", user);
        
    }

    //Logout User
    @SuppressWarnings("unused")
	private void deleteUserFromSession() {
        HttpServletRequest httpServletRequest = this.getThreadLocalRequest();
        HttpSession session = httpServletRequest.getSession();
        session.removeAttribute("user");
    }  
}