import time
import paramiko

class ProcessManager(object):

    def __init__(self, ip, username, password, execute_cmd, exit_cmd):
        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh.connect(ip, username=username, password=password)
        self.execute_cmd = execute_cmd
        self.exit_cmd = exit_cmd

        exit()

    def execute(self):
        self.ssh.exec_command(self.execute_cmd)

    def exit(self):
        self.ssh.exec_command(self.exit_cmd)

    def close(self):
        if ssh is not None:
            ssh.close()


def main(args):
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(args.ip, username=args.username, password=args.password)

    # ssh.exec_command("cd ~/Repository/PnC/build/bin && ./run_cart_pole")
    # ssh.exec_command("pkill cart_pole")

    if ssh is not None:
        ssh.close()

if __name__ == "__main__":

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="localhost")
    parser.add_argument("--username", type=str, default="junhyeokahn")
    parser.add_argument("--password", type=str)
    args = parser.parse_args()

    main(args)
