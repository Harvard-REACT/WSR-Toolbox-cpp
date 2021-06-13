# '''
# (c) REACT LAB, Harvard University
# Author: Ninad Jadhav
# '''

# #!/usr/bin/env python3

import matlab.engine
import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", help="AOA profile csv file")
    args = parser.parse_args()
    
    eng = matlab.engine.start_matlab()
    filename = args.file
    eng.viz_aoa(filename, nargout=0)
    val = input("Finished visualizing?")
    eng.quit()

if __name__ == "__main__":
    main()