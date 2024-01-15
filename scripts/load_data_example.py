#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2024 Marc Bestmann <marc.bestmann@dlr.de>
#
# SPDX-License-Identifier: MIT

"""Example data loader script."""
from argparse import ArgumentParser
import random
import time

from vinspect.vinspect_py import compute_colored_mesh, load, show_colored_mesh

if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('-i', '--input-data-path',
                        help='Path to the input file containing data in vinspect format.')
    args = parser.parse_args()

    start_time = time.time()
    inspection = load(args.input_data_path)
    end_time = time.time()
    print(f'Time taken to load data : {end_time - start_time} s')

    # Exemplarly show how fast we can access the data using the octree
    if True:
        point = (random.random() * 0.1, random.random()
                 * 0.1, random.random() * 0.1)
        ITERATIONS = 1000
        start_time = time.time()
        for i in range(0, ITERATIONS):
            point = (random.random() * 0.1, random.random()
                     * 0.1, random.random() * 0.1)
            inspection.get_closest_sparse_measurement(point)
        end_time = time.time()
        print(
            f'mean time taken for octree closest point call : \
                {(end_time - start_time)/ITERATIONS * 1000} ms')

        start_time = time.time()
        for i in range(0, ITERATIONS):
            point = (random.random() * 0.1, random.random()
                     * 0.1, random.random() * 0.1)
            inspection.get_sparse_measurements_in_radius(point, 0.01)
        end_time = time.time()
        print(
            f'mean time taken for octree radius search call : \
                {(end_time - start_time)/ITERATIONS * 1000} ms')

        # point_sizes = [0.0005, 0.001, 0.005, 0.01, 0.05, 0.1]
        point_sizes = [0.0005]
        for point_size in point_sizes:
            start_time = time.time()
            compute_colored_mesh(inspection, point_size)
            end_time = time.time()
            print(
                f'time taken to compute mesh with point radius \
                    {point_size} : {(end_time - start_time) * 1000} ms')

    # Show the mesh
    print(show_colored_mesh(inspection, 0.0005))
