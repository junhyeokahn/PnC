import sys
import os
sys.path.append(os.getcwd() + '/PnC/ReinforcementLearning')
sys.path.append(os.getcwd() + '/PnC/CartPolePnC')
sys.path.append(os.getcwd() + '/build/Messages')

from CartPoleDataGen import CartPoleDataGen

def main():
    cart_pole_data_gen = CartPoleDataGen('localhost', 'junhyeokahn', args.password, 256)
    cart_pole_data_gen.get_segment()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--password", type=str)
    args = parser.parse_args()
    main()
