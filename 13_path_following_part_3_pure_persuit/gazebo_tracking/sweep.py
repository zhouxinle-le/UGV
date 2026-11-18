#!/usr/bin/env python3
"""Run a route, speed and controller matrix in headless Gazebo."""

from itertools import product

from run import CONTROLLERS, DEFAULT_ROUTE, check_args, parser_for, positive_int, road_file, run_trials


def main():
    parser = parser_for(__doc__)
    parser.add_argument('--route', type=road_file, action='append',
                        help='reference CSV (repeat for multiple routes, default: s_curve.csv)')
    parser.add_argument('--speeds', type=positive_int, nargs='+', default=(5, 10, 15),
                        help='target speeds in km/h (default: 5 10 15)')
    parser.add_argument('--controllers', choices=CONTROLLERS, nargs='+', default=CONTROLLERS)
    parser.add_argument('--repeats', type=positive_int, default=3)
    args = parser.parse_args()
    routes = args.route or [DEFAULT_ROUTE]
    check_args(parser, args, routes)
    trials = product(routes, dict.fromkeys(args.speeds), dict.fromkeys(args.controllers),
                     range(1, args.repeats + 1))
    run_trials(args, routes, trials, 'matrix')


if __name__ == '__main__':
    main()
