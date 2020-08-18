import sys
import select
from time import sleep

sys.stderr.write('repeater.py: starting\n')
sys.stderr.flush()
i = 0
while True:
    # next_line = sys.stdin.readline()
    # sys.stdin.flush()
    sys.stdout.write('next_line')
    sys.stdout.flush()
    sleep(1)

sys.stderr.write('repeater.py: exiting\n')
sys.stderr.flush()
