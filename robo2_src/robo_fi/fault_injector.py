#!/usr/bin/env python
import rclpy
import argparse
import sys

from robo_fi.communication import PipelinedRetransmitter
from robo_fi.injection_types import FixedInjector, ScaleInjector, RandomInjector, NullInjector

def get_args():
    p = argparse.ArgumentParser(description='Fault Injection Toolkit for ROS2')

    p.add_argument('-from', help='topic to read from', default='scan', type=str)
    p.add_argument('-to', help='topic to write to', default='scan_fi', type=str)
    p.add_argument('-fi', help='fault injection pipeline (can specify several injection types)', action='append', nargs='+', metavar=('type', 'args'), required=True)

    return vars(p.parse_args())

def injector_factory(i_type, i_args, pipeline):
    if i_type == 'fixed':
        if len(i_args) < 1:
            print(f"Not enough arguments for injection type {i_type}! Expected 1 but got {len(i_args)}.")
            sys.exit(4)
        return FixedInjector(pipeline, float(i_args[0]))
    elif i_type == 'scale':
        if len(i_args) < 1:
            print(f"Not enough arguments for injection type {i_type}! Expected 1 but got {len(i_args)}.")
            sys.exit(4)
        return ScaleInjector(pipeline, float(i_args[0]))
    elif i_type == 'random':
        return RandomInjector(pipeline)
    elif i_type == 'null':
        print('Warning: Null fault injection selected. Please take into consideration how this can affect following pipeline steps.')
        return NullInjector(pipeline)

def main(args=None):

    args = get_args()

    # print(args)
    # print(f"FI pipeline: {args['fi']}")
    
    pipeline = None

    injection_types = {'fixed', 'scale', 'random', 'null'}
    for injection_type, *injection_args in args['fi']:
        if injection_type not in injection_types:
            print(f"{injection_type} is not a valid injection type (one of {injection_types})")
            sys.exit(3)

        pipeline = injector_factory(injection_type, injection_args, pipeline)

    rclpy.init(args=args)

    communicator = PipelinedRetransmitter(pipeline, args['from'], args['to'])
    print("Fault injection system initialized")
    print(f"Retransmitting the messages received from '{args['from']}' to '{args['to']}' after mutation via fault injection pipeline.")
    # Maybe TODO print this in a cool way? (take advantage of the stack like in the mutate method)

    rclpy.spin(communicator)

    communicator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

