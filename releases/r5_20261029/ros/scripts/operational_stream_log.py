"""Incremental audit of a new owned log; bounded buffers, no tail-only PASS.

Legacy OwnedLog.text retains its explicit1MiB cap. Entry points use this
subclass so a long observe session does not discard early child failures or
fail just because normal logs have grown. All prior ownership checks remain.
"""
import os
import stat
from operational_logs import OwnedLog,STOP,ANSI,CHILD,START,CLEAN,DIED,ESCALATED,strict_json

class Audit:
    def __init__(self):
        self.offset=0;self.pending=b'';self.starts={};self.ends={};self.errors=[]
        self.receipts=0;self.proof=None;self.discard=False;self.fallbacks=[]
    def error(self,reason):
        if reason not in self.errors and len(self.errors)<64:self.errors.append(reason)
    def line(self,raw):
        try:line=ANSI.sub('',raw.decode('utf-8')).rstrip('\r\n')
        except UnicodeError:self.error('INVALID_UTF8_LOG');return
        receipt=STOP.fullmatch(line)
        if receipt:
            self.receipts+=1
            try:self.proof=strict_json(receipt.group(1))
            except (ValueError,TypeError):self.error('INVALID_STOP_JSON')
        match=CHILD.fullmatch(line)
        if match is None:
            if any(x in line for x in ('process started with pid','process has died','process has finished cleanly')):self.error('UNPARSED_CHILD_LINE')
            return
        level,label,body=match.groups();begin=START.fullmatch(body);end=CLEAN.fullmatch(body);dead=DIED.fullmatch(body);fallback=ESCALATED.fullmatch(body)
        if begin:
            key=(label,int(begin.group(1)))
            if key in self.starts or level!='INFO':self.error('DUPLICATE_OR_INVALID_START')
            if len(self.starts)>=4096:self.error('TOO_MANY_CHILD_IDENTITIES');return
            self.starts[key]=True
        elif end or dead:
            key=(label,int((end or dead).group(1)))
            if key not in self.starts or key in self.ends:self.error('UNMATCHED_OR_DUPLICATE_CHILD_EXIT')
            if len(self.ends)<4096:self.ends[key]=0 if end and level=='INFO' else int(dead.group(2)) if dead else None
            if dead or level!='INFO':self.error('CHILD_DIED:'+label+':'+str(self.ends.get(key)))
        elif fallback:
            child,timeout_s,source_signal,target_signal=fallback.groups()
            active=[key for key in self.starts if key[0]==label and key not in self.ends]
            if level!='ERROR' or child!=label or len(active)!=1:
                self.error('UNMATCHED_OR_INVALID_CHILD_ESCALATION')
            else:
                event=dict(label=label,pid=active[0][1],timeout_s=float(timeout_s),
                           from_signal=source_signal,to_signal=target_signal)
                self.fallbacks.append(event)
                self.error('CHILD_SIGNAL_ESCALATION:'+label+':'+source_signal+'_TO_'+target_signal)
        elif body.startswith(('process started','process has died','process has finished')):self.error('UNPARSED_CHILD_LINE')
    def feed(self,data):
        # Split before buffering: a malicious/accidental giant single line can
        # never force an unbounded buffer or hide a previously observed error.
        for part in data.splitlines(keepends=True):
            newline=part.endswith(b'\n')
            if len(self.pending)+len(part)>65536:self.error('LOG_LINE_TOO_LONG');self.pending=b'';self.discard=True
            if not self.discard:self.pending+=part
            if newline:
                if not self.discard:self.line(self.pending)
                self.pending=b'';self.discard=False
    def report(self,final=False):
        errors=list(self.errors)
        if final and self.pending:errors.append('INCOMPLETE_LOG_LINE')
        missing=[dict(label=k[0],pid=k[1]) for k in self.starts if k not in self.ends]
        return dict(clean=bool(self.starts) and not errors and not missing,started=len(self.starts),terminal=len(self.ends),missing=missing,errors=errors,
                    internal_signal_fallback=bool(self.fallbacks),fallbacks=list(self.fallbacks),
                    scope='OWNED_LAUNCH_CHILD_LOG_WITH_INTERNAL_FALLBACK_AUDIT_NO_PHYSICAL_STOP_CLAIM')


class StreamingOwnedLog(OwnedLog):
    def __init__(self,path):super().__init__(path);self.audit_state=Audit()
    def scan(self,*,final=False):
        p=self.process
        if p is None:raise ValueError('NO_OWNED_PROCESS')
        if final and (not p.done or not p.report()['group_exit_confirmed']):raise ValueError('OWNED_GROUP_NOT_EXITED')
        current=os.fstat(self.file.fileno());named=self.path.stat()
        if (not stat.S_ISREG(current.st_mode) or self.path.is_symlink()
            or (current.st_dev,current.st_ino)!=(named.st_dev,named.st_ino)
            or (current.st_dev,current.st_ino)!=(self.identity.st_dev,self.identity.st_ino)):
            raise ValueError('OWNED_LOG_REPLACED')
        if current.st_size<self.audit_state.offset:raise ValueError('OWNED_LOG_TRUNCATED')
        while self.audit_state.offset<current.st_size:
            raw=os.pread(self.file.fileno(),min(65536,current.st_size-self.audit_state.offset),self.audit_state.offset)
            if not raw:raise ValueError('OWNED_LOG_READ_INCOMPLETE')
            self.audit_state.offset+=len(raw);self.audit_state.feed(raw)
        return self.audit_state.report(final=final)
    def receipt(self):
        p=self.process
        if p is None or p.rc!=0 or p.forced or p.error is not None:raise ValueError('OPERATOR_EXIT_UNCLEAN')
        self.scan(final=True);a=self.audit_state
        if a.receipts!=1 or a.errors or a.pending or not isinstance(a.proof,dict):raise ValueError('EXACTLY_ONE_OWNED_COMPLETE_STOP_RECEIPT_REQUIRED')
        return a.proof
    def child_audit(self):return self.scan(final=True)
