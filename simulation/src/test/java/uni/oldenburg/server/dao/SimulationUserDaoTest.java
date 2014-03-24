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
}
