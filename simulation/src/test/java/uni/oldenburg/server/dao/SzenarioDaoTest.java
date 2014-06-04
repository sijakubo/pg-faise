	package uni.oldenburg.server.dao;

import junit.framework.Assert;

import org.junit.Ignore;
import org.junit.Test;

import uni.oldenburg.shared.model.Szenario;

import java.sql.SQLException;

public class SzenarioDaoTest {
	@Ignore
    @Test
    public void testLoadSzenario() {
		SzenarioDao szenarioDao = new SzenarioDao();
        
        try {
            Szenario testSzenario = szenarioDao.loadSzenario("TestSzenario");
            Assert.assertNotNull(testSzenario);
        } catch (SQLException e) {
            e.printStackTrace();
            Assert.fail();
        }	
    }
}
