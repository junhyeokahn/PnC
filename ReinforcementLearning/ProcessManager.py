import time
import paramiko

class ProcessManager(object):

    def __init__(self, ip, username, password, execute_cmd, exit_cmd):

        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh.connect(ip, username=username, password=password)
        self.execute_cmd = execute_cmd
        self.exit_cmd = exit_cmd

        self.quit_process()

    def execute_process(self):
        # print("[[Execute Process]]")
        self.ssh.exec_command(self.execute_cmd)
        time.sleep(0.5)

    def quit_process(self):
        # print("[[Quit Process]]")
        self.ssh.exec_command(self.exit_cmd)
        time.sleep(0.5)

    def close(self):
        if ssh is not None:
            ssh.close()
