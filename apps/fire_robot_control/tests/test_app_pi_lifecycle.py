"""R5 vs app Pi state machines with fake I/O, including bounded error drain."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'releases/r5_20261029/ros/scripts'))
from r5_pi_session import PiController
from operational_entry import StartupSequence
from services.app_pi_lifecycle import AppPiController,DRAIN_SECONDS

class PiLifecycleTests(unittest.TestCase):
    def controller(self,klass=AppPiController):
        self.calls=[];self.current={'io':False}
        return klass({'session':'test'},lambda:self.calls.append('start'),lambda:self.calls.append('stop'),
                     lambda:dict(remote_groups_exited=True,children_clean=True,forced=False),lambda:dict(self.current),0.)
    def seal_wait(self,klass=AppPiController):
        c=self.controller(klass);c.command('START',0.);c.command('SEAL',1.);return c
    def test_app_timeout_holds_io_until_laptop_stop_original_r5_unchanged(self):
        original=self.seal_wait(PiController);original.tick(21.)
        self.assertEqual(original.phase,'DONE');self.assertIn('stop',self.calls)
        c=self.seal_wait();c.tick(21.)
        self.assertEqual(c.phase,'WAIT_LAPTOP_DRAIN');self.assertNotIn('stop',self.calls)
        self.assertEqual(c.command('STATE',22.)['error'],'PI_SEAL_HEALTH_TIMEOUT')
        c.command('STOP',23.);c.tick(23.)
        self.assertEqual(c.phase,'DONE');self.assertEqual(self.calls,['start','stop'])
    def test_missing_laptop_drain_expires_once_and_cannot_reopen(self):
        c=self.seal_wait();c.tick(21.);c.tick(21.+DRAIN_SECONDS-.001)
        self.assertEqual(c.phase,'WAIT_LAPTOP_DRAIN')
        for command in ('START','RETRY_PREARM','SEAL'):
            with self.assertRaises(ValueError):c.command(command,30.)
        c.tick(21.+DRAIN_SECONDS)
        self.assertEqual(self.calls.count('stop'),1);self.assertEqual(c.phase,'DONE')
        c.command('STOP',100.);self.assertEqual(self.calls.count('stop'),1)
    def test_good_seal_not_delayed_and_readiness_still_current(self):
        c=self.seal_wait();self.current['io']=True;c.tick(2.)
        self.assertTrue(c.sealed);self.assertEqual(c.phase,'RUNNING_OBSERVE')
        self.current['io']=False
        self.assertFalse(c.command('STATE',3.)['health']['io'])
    def test_prearm_retry_once_and_not_after_seal(self):
        c=self.controller();c.command('START',0);c.command('RETRY_PREARM',1);c.tick(2)
        self.assertEqual(c.retries,1)
        with self.assertRaises(ValueError):c.command('RETRY_PREARM',3)
        c.command('SEAL',4)
        with self.assertRaises(ValueError):c.command('RETRY_PREARM',5)
    def test_fatal_clock_regression_does_not_reset_deadlines(self):
        c=self.seal_wait();c.tick(3)
        with self.assertRaises(ValueError):c.tick(2)
        s=StartupSequence(0.);s.step(0.,{'clock_ok':True});s.step(60.,{})
        self.assertEqual(s.error,'STARTUP_TIMEOUT:PI_IO')
    def test_sealed_but_unhealthy_does_not_start_nav_and_no_deadline_extension(self):
        s=StartupSequence(0.,live=True)
        s.step(0.,{'clock_ok':True});s.step(1.,{'io':True});s.step(2.,{'io':True,'camera':True})
        for when in (3.,10.,20.):
            self.assertEqual(s.step(when,dict(pi_sealed=True,io=False,camera=True)),[])
        self.assertEqual(s.step(22.,dict(pi_sealed=True,io=True,camera=True)),['STOP'])
    def test_unchanged_startup_effects_do_not_admit_pose_yolo_or_consumers_early(self):
        s=StartupSequence(0.,live=True)
        self.assertEqual(s.step(0.,dict(clock_ok=True)),['START_PI','START_HEALTH'])
        s.step(1.,dict(io=True));self.assertEqual(s.stage,'CAMERA')
        self.assertEqual(s.step(2.,dict(io=True,camera=True)),['SEAL_PI'])
        self.assertEqual(s.step(3.,dict(io=True,camera=True,pi_sealed=True)),['START_NAV','START_RVIZ'])
        self.assertEqual(s.step(4.,dict(camera=True,pose=False)),[])
