"""Exercise real callback bodies with transport doubles, without claiming ROS integration."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
from kickboard_robot.extended_policy import admission
from kickboard_robot.mission_logic import Cargo


def cargo(*items):
    """미션 관리자의 적재함 (D 의 mission_logic.Cargo)"""
    c = Cargo()
    for sid, dest in items:
        c.add(sid, dest)
    return c


def method(cls,name,**env):
    path=Path(__file__).parents[1]/'kickboard_robot/extended_runtime.py'
    tree=ast.parse(path.read_text(encoding='utf-8'))
    c=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name==cls)
    fn=next(n for n in c.body if isinstance(n,ast.FunctionDef) and n.name==name)
    scope=dict(admission=admission,SELECTING='SELECTING',String=lambda **kw:NS(**kw),**env)
    exec(compile(ast.Module(body=[fn],type_ignores=[]),str(path),'exec'),scope)
    return scope[name]


def test_dock_exhaustion_never_charges():
    f=NS(result=lambda:NS(result=NS(success=False)))
    m=NS(p=lambda _:3,pending=True,dock_fail=2,at_dock=False,ev=NS(emit=Mock()),
         alert=Mock(),set_state=Mock(),start_dock=Mock())
    method('ExtendedMission','on_dock_done')(m,f)
    assert not m.at_dock and not m.pending
    m.set_state.assert_called_once_with('NEEDS_HUMAN','(도킹 실패)')
    m.start_dock.assert_not_called()


def test_dock_retries_before_exhaustion():
    m=NS(p=lambda _:3,pending=True,dock_fail=0,ev=NS(emit=Mock()),start_dock=Mock())
    method('ExtendedMission','on_dock_done')(m,NS(result=lambda:NS(result=NS(success=False))))
    assert m.dock_fail==1
    m.start_dock.assert_called_once()


def test_damage_short_circuits_collection():
    m=NS(specs={'1':{'damaged':True}},cargo=cargo(),weights={},weight_limit=75,
         report=Mock(),alert=Mock(),ev=NS(emit=Mock()),set_state=Mock())
    method('ExtendedMission','start_collect')(m,'1',None)
    assert m.report.call_args.args[1]=='NEEDS_HUMAN'


def test_overweight_requeues_before_unload():
    m=NS(specs={'2':{'weight_kg':32}},cargo=cargo(('1','parking')),weights={'1':32},weight_limit=50,
         report=Mock(),ev=NS(emit=Mock()),start_drop=Mock(return_value=True),start_dock=Mock())
    method('ExtendedMission','start_collect')(m,'2',None)
    assert m.report.call_args.args[1]=='REQUEUE'
    m.start_drop.assert_called_once()
    m.start_dock.assert_not_called()


def test_cancel_waits_for_action_result():
    m=NS(cancel_target='1',target='1',collect_handle=object(),report=Mock(),ev=NS(emit=Mock()),set_state=Mock())
    method('ExtendedMission','on_collect_done')(m,NS(result=lambda:NS(result=NS(success=False))))
    assert m.target is None and m.collect_handle is None
    assert m.report.call_args.args[1]=='CANCELLED'
    m.set_state.assert_called_once_with('SELECTING')


def test_queue_rejects_cancel_during_loading():
    m=NS(items={'1':NS(status='IN_PROGRESS')},robot_target='1',robot_phase='LOADING')
    res=NS()
    method('ExtendedQueue','on_cancel',ql=NS(IN_PROGRESS='IN_PROGRESS'))(m,NS(scooter_id='1'),res)
    assert res.success is False
