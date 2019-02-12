import time
import paramiko

class ProcessManager(object):

    def __init__(self, ip, username, password, execute_cmd, exit_cmd, verbose=0):

        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh.connect(ip, username=username, password=password)
        self.execute_cmd = execute_cmd
        self.exit_cmd = exit_cmd
        self.verbose = verbose

        self.quit_process()

    def execute_process(self):
        time.sleep(0.5)
        if self.verbose >= 1:
             print("[[Execute Process]]")
        self.ssh.exec_command(self.execute_cmd)

    def quit_process(self):
        if self.verbose >= 1:
            print("[[Quit Process]]")
        self.ssh.exec_command(self.exit_cmd)
        time.sleep(0.5)

    def close(self):
        if ssh is not None:
            ssh.close()
