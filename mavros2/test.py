import subprocess

proc = subprocess.Popen(
                        'ping 8.8.8.8',
                        # 'python3 repeater.py',
                        # 'mavproxy.py',
                        shell=True,
                        stdin=subprocess.PIPE,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        universal_newlines=True,
                        bufsize=5
                        )



out, _ = proc.communicate(timeout=1)