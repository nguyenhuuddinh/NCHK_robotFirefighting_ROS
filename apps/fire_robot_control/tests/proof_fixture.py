"""Structured synthetic R5 receipts, never hardware STOP evidence."""
def group(pid=90001):
    return dict(pid=pid,done=True,phase='GROUP_EXITED',group_exit_confirmed=True,forced=False,
                leader_rc=0,error=None,live_pids=[])
def children():
    return dict(complete=True,clean=True,roles={'fixture':dict(clean=True,started=1,terminal=1,
                missing=[],errors=[],internal_signal_fallback=False)})
def clean_local():
    return dict(stop_confirmed=True,clean=True,group_cleanup_clean=True,errors=[],
                processes={'operator':group()},child_log_audit=children())
def clean_remote(binding):
    return dict(confirmed=True,binding=binding,receipt=dict(remote_groups_exited=True,
        children_clean=True,forced=False,groups={'pi':group(90002)},child_audit=children()))
