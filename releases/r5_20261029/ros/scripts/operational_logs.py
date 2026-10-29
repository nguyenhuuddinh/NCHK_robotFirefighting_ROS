"""Bind receipts/child exits to a freshly created log of an owned process.

No ROS, network, signal or automatic process start. The caller explicitly starts
its OwnedProcess through OwnedLog.start; pre-existing logs are never accepted.
"""
import json
import os
from pathlib import Path
import re
import stat
from operational_owned_process import OwnedProcess

ANSI=re.compile(r'\x1b\[[0-9;]*m')
STOP=re.compile(r'^(?:\[fire_robot_operator-\d+\] )?\[INFO\] \[\d+\.\d+\] \[fire_robot_operator\]: OPERATOR_STOP (\{.*\})$')
CHILD=re.compile(r'^\[(INFO|ERROR)\] \[([^\]]+)\]: (.+)$')
START=re.compile(r'^process started with pid \[(\d+)\]$')
CLEAN=re.compile(r'^process has finished cleanly \[pid (\d+)\]$')
DIED=re.compile(r'^process has died \[pid (\d+), exit code (-?\d+), cmd .*\]\.?$')
ESCALATED=re.compile(
    r"^process\[([^\]]+)\] failed to terminate '([0-9]+(?:\.[0-9]+)?)' seconds "
    r"after receiving '(SIGINT|SIGTERM)', escalating to '(SIGTERM|SIGKILL)'$")


def strict_json(text):
    def pairs(rows):
        result={}
        for key,value in rows:
            if key in result:raise ValueError('DUPLICATE_JSON_KEY')
            result[key]=value
        return result
    def invalid(value):raise ValueError('NONFINITE_JSON')
    return json.loads(text,object_pairs_hook=pairs,parse_constant=invalid)


def operator_receipt(text):
    receipts=[]
    for line in ANSI.sub('',text).splitlines(keepends=True):
        if not line.endswith('\n'):continue
        match=STOP.fullmatch(line.rstrip('\r\n'))
        if match:
            value=strict_json(match.group(1))
            if not isinstance(value,dict):raise ValueError('INVALID_STOP_RECEIPT')
            receipts.append(value)
    if len(receipts)!=1:raise ValueError('EXACTLY_ONE_OWNED_OPERATOR_RECEIPT_REQUIRED')
    return receipts[0]


def audit_launch_children(text):
    started={};terminal={};errors=[];fallbacks=[]
    for raw in ANSI.sub('',text).splitlines(keepends=True):
        if not raw.endswith('\n'):
            if 'process ' in raw:errors.append('INCOMPLETE_CHILD_LINE')
            continue
        line=raw.rstrip('\r\n');match=CHILD.fullmatch(line)
        if match is None:
            if 'process started with pid' in line or 'process has died' in line or 'process has finished cleanly' in line:
                errors.append('UNPARSED_CHILD_LINE')
            continue
        level,label,body=match.groups();begin=START.fullmatch(body);end=CLEAN.fullmatch(body);dead=DIED.fullmatch(body);fallback=ESCALATED.fullmatch(body)
        if begin:
            key=(label,int(begin.group(1)))
            if level!='INFO' or key in started:errors.append('DUPLICATE_OR_INVALID_START')
            started[key]=True
        elif end or dead:
            key=(label,int((end or dead).group(1)))
            if key not in started or key in terminal:errors.append('UNMATCHED_OR_DUPLICATE_CHILD_EXIT')
            terminal[key]=0 if end and level=='INFO' else int(dead.group(2)) if dead else None
            if dead or level!='INFO':errors.append('CHILD_DIED:'+label+':'+str(terminal[key]))
        elif fallback:
            child,timeout_s,source_signal,target_signal=fallback.groups()
            active=[key for key in started if key[0]==label and key not in terminal]
            if level!='ERROR' or child!=label or len(active)!=1:
                errors.append('UNMATCHED_OR_INVALID_CHILD_ESCALATION')
            else:
                event=dict(label=label,pid=active[0][1],timeout_s=float(timeout_s),
                           from_signal=source_signal,to_signal=target_signal)
                fallbacks.append(event)
                errors.append('CHILD_SIGNAL_ESCALATION:'+label+':'+source_signal+'_TO_'+target_signal)
        elif body.startswith(('process started','process has died','process has finished')):
            errors.append('UNPARSED_CHILD_LINE')
    missing=[dict(label=label,pid=pid) for label,pid in started if (label,pid) not in terminal]
    clean=bool(started) and not errors and not missing and all(v==0 for v in terminal.values())
    return dict(clean=clean,started=len(started),terminal=len(terminal),missing=missing,errors=errors,
                internal_signal_fallback=bool(fallbacks),fallbacks=fallbacks,
                scope='OWNED_LAUNCH_CHILD_LOG_WITH_INTERNAL_FALLBACK_AUDIT_NO_PHYSICAL_STOP_CLAIM')


class OwnedLog:
    def __init__(self,path):
        self.path=Path(path)
        fd=os.open(self.path,os.O_RDWR|os.O_CREAT|os.O_EXCL,0o600)
        self.file=os.fdopen(fd,'w+b',buffering=0);self.identity=os.fstat(fd)
        self.process=None

    def start(self,process,argv,now=None):
        if self.process is not None or not isinstance(process,OwnedProcess) or process.process is not None:
            raise ValueError('ONE_FRESH_OWNED_PROCESS_REQUIRED')
        self.process=process
        return process.start(argv,self.file,now)

    def text(self):
        p=self.process
        if p is None or not p.done or not p.report()['group_exit_confirmed']:
            raise ValueError('OWNED_GROUP_NOT_EXITED')
        current=os.fstat(self.file.fileno());named=self.path.stat()
        if not stat.S_ISREG(current.st_mode) or (current.st_dev,current.st_ino)!=(named.st_dev,named.st_ino):
            raise ValueError('OWNED_LOG_REPLACED')
        if (current.st_dev,current.st_ino)!=(self.identity.st_dev,self.identity.st_ino):raise ValueError('OWNED_LOG_CHANGED')
        # Never silently omit an earlier duplicate/failed child from the audit.
        if current.st_size>1024*1024:raise ValueError('OWNED_LOG_AUDIT_LIMIT_EXCEEDED')
        raw=os.pread(self.file.fileno(),current.st_size,0)
        return raw.decode('utf-8')

    def receipt(self):
        p=self.process
        if p is None or p.rc!=0 or p.forced or p.error is not None:raise ValueError('OPERATOR_EXIT_UNCLEAN')
        return operator_receipt(self.text())

    def child_audit(self):
        text=self.text()
        if self.path.stat().st_size>1024*1024:raise ValueError('CHILD_AUDIT_NEEDS_COMPLETE_LOG')
        return audit_launch_children(text)

    def close(self):self.file.close()


class OwnedLogRegistry:
    def __init__(self):self.logs={}
    def register(self,role,log):
        if role in self.logs or not isinstance(log,OwnedLog) or log.process is None:
            raise ValueError('ONE_STARTED_LOG_PER_ROLE_REQUIRED')
        if any(old.process is log.process or old.file.fileno()==log.file.fileno() for old in self.logs.values()):
            raise ValueError('LOG_OR_PROCESS_REGISTERED_TWICE')
        self.logs[role]=log
    def matches(self,role,process):return role in self.logs and self.logs[role].process is process
    def audit(self,processes):
        if set(processes)!=set(self.logs) or not all(self.matches(r,p) for r,p in processes.items()):
            raise ValueError('LOG_REGISTRY_PROCESS_MISMATCH')
        rows={r:self.logs[r].child_audit() for r,p in processes.items() if p.launch_leader}
        return dict(complete=True,clean=all(x['clean'] for x in rows.values()),roles=rows)
