# Processes that may crash or lag without blocking openpilot engagement.
IGNORED_PROCESSES = frozenset({'mapd'})

# Grace period after onroad start before process health blocks engage.
ONROAD_PROCESS_GRACE_SEC = 3.0
