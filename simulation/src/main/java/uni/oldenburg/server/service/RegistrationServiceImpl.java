package uni.oldenburg.server.service;

import com.google.gwt.user.server.rpc.RemoteServiceServlet;
import uni.oldenburg.client.service.RegistrationService;
import uni.oldenburg.server.dao.SimulationUserDao;
import uni.oldenburg.shared.model.SimulationUser;

import java.sql.SQLException;

public class RegistrationServiceImpl extends RemoteServiceServlet implements RegistrationService {
    @Override
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
}
