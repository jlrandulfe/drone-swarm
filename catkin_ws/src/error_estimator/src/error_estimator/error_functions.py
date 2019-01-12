#!/usr/bin/env python3
"""
Error estimator top level module
"""
# Standard libraries
# Third-party libraries
import numpy as np
# Local libraries


def simple_differences(desired_distances, actual_distances):
    """
    Returns as error the subtraction between desired and actual values
    """
    errors = desired_distances - actual_distances
    return errors
