"""Shared static launch contracts; runtime behavior is tested separately."""
from pathlib import Path
def shared_shutdown(launch):
    path=Path(__file__).resolve().parents[1]/'scripts/process_helpers.sh'
    if 'source "$REPO_DIR/scripts/process_helpers.sh"' not in launch or not path.is_file():
        return False
    helper=path.read_text()
    return all(token in helper for token in (
        'stop_group()', 'kill -TERM -- "-$group"', 'kill -KILL -- "-$group"',
        'seq 1 20', 'wait "$group"'))
