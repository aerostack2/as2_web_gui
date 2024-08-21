# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__authors__ = 'Rafael Pérez Seguí'

""" Interface for path and distribution algorithms """


class CommonAlgorithm_utils:
    """ Interface for common algorithms utils """

    @staticmethod
    def _extract_height_from_list(position_list: list) -> list:
        """ Extract the height from a list of positions
        Args:
            position_list (list): List of lists with x, y and height values
        Returns:
            position_list (list): List of lists with x and y values
            height_list (list): List of height values
        """
        height_list = []
        position_without_height = []
        for position in position_list:
            x, y, z = position
            position_without_height.append([x, y])
            height_list.append(z)
        return position_without_height, height_list

    @staticmethod
    def _add_height_list_to_position_list(position_list: list, height_list: list) -> list:
        """ Add the height to a list of positions
        Args:
            position_list (list): List of lists with x and y values
            height_list (list): List of height values
        Returns:
            position_list (list): List of lists with x, y and height values
        """
        if len(position_list) != len(height_list):
            raise ValueError('The length of the position list and the height list must be the same')
        position_with_height = []
        for position, height in zip(position_list, height_list):
            x, y = position
            position_with_height.append([x, y, height])
        return position_with_height
    
    @staticmethod
    def _add_height_to_position_list(position_list: list, height: float) -> list:
        """ Add the height to a list of positions
        Args:
            position_list (list): List of lists with x and y values
            height (float): Height value
        Returns:
            position_list (list): List of lists with x, y and height values
        """
        position_with_height = []
        for position in position_list:
            x, y = position
            position_with_height.append([x, y, height])
        return position_with_height


class PathAlgorithm(CommonAlgorithm_utils):
    """ Interface for path algorithms """

    @staticmethod
    def generate_path(
            initial_position_list: list,
            last_position_list: list,
            area_values: list,
            street_spacing: float,
            waypoints_spacing: float,
            theta: float = None) -> list:
        """ Algorithm to transform the area into a grid of waypoints with a sequence of streets
        Args:
            initial_position_list (list): List of lists with x, y and height values of the initial
                position of each UAV
            last_position_list (list): List of lists with x, y and height values of the last
                position of each UAV
            area_values (list): List of vertexes with x, y and height values of the area
            street_spacing (float): Space between streets
            waypoints_spacing (float): Space between waypoints
            theta (float): Angle of the area (in degrees). Default 'None' is automatic selected
        Returns:
            path_waypoints (list): List of lists with x, y and height values of the path to
                cover the area
        """
        raise NotImplementedError

class DistributionAlgorithm(CommonAlgorithm_utils):
    """ Interface for distribution algorithms """

    @staticmethod
    def distribute_path(
            initial_position_list: list,
            last_position_list: list,
            path_waypoints: list,
            uavs_weight_list: list = None) -> list:
        """ Algorithm to distribute the waypoints of the area to each UAV
        Args:
            initial_position_list (list): List of lists with x, y and height values of the initial
                position of each UAV
            last_position_list (list): List of lists with x, y and height values of the last
                position of each UAV
            path_waypoints (list): List of lists with x, y and height values of the path to
                cover the area
            uavs_weight_list (list): List of weights for each UAV. Default 'None' is equal weights
                for all UAVs
        Returns:
            uavs_path (list): List of lists with x, y and height values of the path for each UAV
        """
        raise NotImplementedError
