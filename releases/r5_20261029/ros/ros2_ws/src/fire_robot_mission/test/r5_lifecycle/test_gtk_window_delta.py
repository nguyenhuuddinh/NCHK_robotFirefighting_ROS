"""Actual SessionView/operator main with fake HighGUI; no window or ROS context."""
import math
import signal
from types import SimpleNamespace as N
import numpy as np
import pytest
from fire_robot_mission.operational_view import SessionView
from fire_robot_mission import operational_session as ros
from harness import Robot


class FakeHighGUI:
    WND_PROP_VISIBLE=4
    WND_PROP_AUTOSIZE=1
    FONT_HERSHEY_SIMPLEX=0
    def __init__(self,autosize=1):
        self.autosize=autosize;self.exists=False;self.shows=0;self.keys=[];self.props=[];self.text=[]
    def getWindowProperty(self,name,prop):
        self.props.append(prop)
        if prop==self.WND_PROP_VISIBLE:return -1.  # OpenCV4.5.4 GTK behavior
        assert prop==self.WND_PROP_AUTOSIZE
        return self.autosize if self.exists else -1.
    def imshow(self,*args):self.exists=True;self.shows+=1
    def putText(self,panel,text,*args):self.text.append(text)
    def waitKey(self,*args):return self.keys.pop(0) if self.keys else -1
    def destroyAllWindows(self):self.exists=False


def view_with(gui):
    view=object.__new__(SessionView)
    view.cv2=gui;view.np=np;view.name='test';view.visible=False
    view.image=None;view.image_at=None
    return view


@pytest.mark.parametrize('autosize',[0.,1.])
def test_gtk_unsupported_visible_does_not_close_existing_window(autosize):
    gui=FakeHighGUI(autosize);v=view_with(gui)
    for _ in range(20):assert v.tick({}) is None
    assert gui.shows==20 and set(gui.props)=={gui.WND_PROP_AUTOSIZE}
    assert gui.getWindowProperty(v.name,gui.WND_PROP_VISIBLE)==-1
    assert any('STALE / NO IMAGE' in t for t in gui.text)


@pytest.mark.parametrize('code,key',[(ord('q'),'Q'),(ord('Q'),'Q'),(27,'ESC'),(ord('x'),'X'),(ord('a'),'A'),(ord('p'),'P')])
def test_keys_work_on_gtk_without_camera(code,key):
    gui=FakeHighGUI();v=view_with(gui);v.tick({})
    gui.keys=[code];assert v.tick({})==key


@pytest.mark.parametrize('autosize',[0.,1.])
def test_real_close_is_reported_before_imshow_and_never_reopens(autosize):
    gui=FakeHighGUI(autosize);v=view_with(gui);v.tick({});gui.exists=False
    for _ in range(3):assert v.tick({})=='WINDOW_CLOSED'
    assert gui.shows==1 and not gui.exists


@pytest.mark.parametrize('value',[math.nan,math.inf,-math.inf])
def test_invalid_window_probe_is_explicit_error_not_ignored(value):
    gui=FakeHighGUI(value);v=view_with(gui);v.tick({})
    with pytest.raises(RuntimeError,match='GUI_WINDOW_EXISTENCE_PROBE_INVALID'):v.tick({})
    assert gui.shows==1


def test_probe_exception_propagates_to_operator_cleanup():
    gui=FakeHighGUI();v=view_with(gui);v.tick({})
    gui.getWindowProperty=lambda *a:(_ for _ in ()).throw(RuntimeError('fake HighGUI failure'))
    with pytest.raises(RuntimeError,match='HighGUI failure'):v.tick({})
    assert gui.shows==1


@pytest.mark.parametrize('end',['Q','ESC','WINDOW_CLOSED','SIGINT','probe_error'])
def test_actual_operator_main_waits_on_gtk_then_drains_real_stop_chain(monkeypatch,end):
    h=Robot(monkeypatch);gui=FakeHighGUI();v=view_with(gui);h.op.view=v
    # Use real six-adapter bus/state/STOP checker, but all I/O and signals fake.
    def destroy():
        assert h.op.stop_verification.done and h.op.stop_verification.confirmed
    monkeypatch.setattr(h.op,'destroy_node',destroy)
    monkeypatch.setattr(ros,'OperationalSessionNode',lambda:h.op)
    active=[False];handlers={};calls=[0]
    monkeypatch.setattr(ros.signal,'getsignal',lambda *a:'saved')
    monkeypatch.setattr(ros.signal,'signal',lambda s,fn:handlers.update({s:fn}))
    monkeypatch.setattr(ros.rclpy,'init',lambda **kw:active.__setitem__(0,True))
    monkeypatch.setattr(ros.rclpy,'ok',lambda:active[0])
    monkeypatch.setattr(ros.rclpy,'shutdown',lambda:active.__setitem__(0,False))
    def spin(node,timeout_sec):
        calls[0]+=1
        h.bus.advance();h.inputs();h.bus.flush()
        for n in h.nodes:
            for cb in n.test_timers:cb()
            h.bus.flush()
        if calls[0]<40:
            assert h.op.stop_verification.start is None, 'GTK unsupported property caused early shutdown'
        if calls[0]==40:
            if end=='Q':gui.keys=[ord('q')]
            elif end=='ESC':gui.keys=[27]
            elif end=='WINDOW_CLOSED':gui.exists=False
            elif end=='SIGINT':handlers[signal.SIGINT](None,None)
            else:gui.autosize=math.nan
        assert calls[0]<180
    monkeypatch.setattr(ros.rclpy,'spin_once',spin)
    assert ros.main()==(2 if end=='probe_error' else 0)
    assert calls[0]>=40 and gui.shows>=39
    assert h.op.stop_verification.done and h.op.stop_verification.confirmed
    assert not h.op.core.armed and not active[0] and not gui.exists
    assert not h.pulses and h.x==0 and h.yaw==0
