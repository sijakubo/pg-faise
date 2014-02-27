package uni.oldenburg.server.service;

import com.google.gwt.user.server.rpc.RemoteServiceServlet;

import uni.oldenburg.client.service.RegistrationAndLoginService;
import uni.oldenburg.server.dao.SimulationUserDao;
import uni.oldenburg.shared.model.SimulationUser;

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

    public boolean loginUser(String username, String password) {
        SimulationUserDao simulationUserDao = new SimulationUserDao();
        boolean matchingUsernameAndPassword = false;
        
        try {
            matchingUsernameAndPassword = simulationUserDao.isCredentialsCorrect(username, password);
        } catch (SQLException e) {
            e.printStackTrace();
        }

        return matchingUsernameAndPassword;
    }
}
