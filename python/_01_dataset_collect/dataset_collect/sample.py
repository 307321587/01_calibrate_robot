from typing import Union, List,Optional

import numpy as np
from mathutils import Vector


def shell_section(center: Union[Vector, np.ndarray, List[float]], radius_min: float, radius_max: float, radius_section: int = 10,
          elevation_min: float = -90, elevation_max: float = 90,elevation_section: int = 10, azimuth_min: float = -180,
          azimuth_max: float = 180,azimuth_section: int = 10, uniform_volume: bool = False) -> np.ndarray:
    """
    x轴为正
    Samples a point from the volume between two spheres (radius_min, radius_max). Optionally the spheres can
    be constraint by setting elevation and azimuth angles. E.g. if you only want to sample in the upper
    hemisphere set elevation_min = 0.

    :param center: Center shared by both spheres.
    :param radius_min: Radius of the smaller sphere.
    :param radius_max: Radius of the bigger sphere.
    :param elevation_min: Minimum angle of elevation in degrees. Range: [-90, 90].
    :param elevation_max: Maximum angle of elevation in degrees. Range: [-90, 90].
    :param azimuth_min: Minimum angle of azimuth in degrees. Range: [-180, 180].
    :param azimuth_max: Maximum angle of azimuth in degrees. Range: [-180, 180].
    :param uniform_volume: Instead of sampling the angles and radius uniformly, sample the shell volume uniformly.
                           As a result, there will be more samples at larger radii.
    :return: A sampled point.
    """

    center = np.array(center)

    assert -180 <= azimuth_min <= 180, "azimuth_min must be in range [-180, 180]"
    assert -180 <= azimuth_max <= 180, "azimuth_max must be in range [-180, 180]"
    assert -90 <= elevation_min <= 90, "elevation_min must be in range [-90, 90]"
    assert -90 <= elevation_min <= 90, "elevation_max must be in range [-90, 90]"
    assert azimuth_min < azimuth_max, "azimuth_min must be smaller than azimuth_max"
    assert elevation_min < elevation_max, "elevation_min must be smaller than elevation_max"

    radius_list=np.linspace(radius_min,radius_max,radius_section)
    elevation_list=np.linspace(elevation_min,elevation_max,elevation_section)
    azimuth_list=np.linspace(azimuth_min,azimuth_max,azimuth_section)
    position_list=[]

    for radius_sampled in radius_list:
        for el_sampled_deg in elevation_list:
            for az_sampled_deg in azimuth_list:
                el_sampled=np.deg2rad(el_sampled_deg)
                az_sampled=np.deg2rad(az_sampled_deg)
                direction_vector = np.array([np.sin(np.pi / 2 - el_sampled) * np.cos(az_sampled),
                                                np.sin(np.pi / 2 - el_sampled) * np.sin(az_sampled),
                                                np.cos(np.pi / 2 - el_sampled)])

                # Get the coordinates of a sampled point inside the shell
                position = direction_vector * radius_sampled + center
                position_list.append(position)

    return np.array(position_list)



def random_walk(total_length: int, dims: int, step_magnitude: float = 1.0, window_size: int = 1,
                interval: Optional[List[np.ndarray]] = None, distribution: str = 'uniform',
                order: float = 1.0) -> np.ndarray:
    """
    Creates a random walk with the specified properties. Can be used to simulate camera shaking or POI drift.
    steps ~ step_magnitude * U[-1,1]^order

    :param total_length: length of the random walk
    :param dims: In how many dimensions the random walk should happen
    :param step_magnitude: Maximum magnitude of any coordinate in a single step
    :param window_size: Convolve the final trajectory with an average filter that smoothens the trajectory with a
                        given filter size.
    :param interval: Constrain the random walk to an interval and mirror steps if they go beyond. List of arrays
                     with dimension dims.
    :param distribution: Distribution to sample steps from. Choose from ['normal', 'uniform'].
    :param order: Sample from higher order distribution instead of the uniform. Higher order leads to steps being
                  less frequently close to step_magnitude and thus overall decreased variance.
    :return: The random walk trajectory (total_length, dims)
    """
    # Set sampling distribution
    if distribution == 'uniform':
        dist_fun = np.random.rand
    elif distribution == 'normal':
        dist_fun = np.random.randn
    else:
        raise RuntimeError(f'Unknown distribution: {distribution}. Choose between "normal" and "uniform"')

        # Sample random steps
    random_steps = step_magnitude * np.random.choice([-1, 1], (total_length, dims)) * dist_fun(total_length,
                                                                                               dims) ** order

    # Cumulate the random steps to a random walk trajectory
    cumulative_steps = np.cumsum(random_steps, axis=0)

    # Keep the steps within the predefined interval
    if interval is not None:
        assert len(interval) == 2, "interval must have length of two"
        left_bound = np.array(interval[0])
        size = np.abs(interval[1] - left_bound)
        cumulative_steps = np.abs((cumulative_steps - left_bound + size) % (2 * size) - size) + left_bound

    # Smooth the random walk trajectory using a sliding window of size window_size
    if window_size > 1:
        initial_padding = np.ones((window_size - 1, dims)) * cumulative_steps[:1, :]
        cumulative_steps_padded = np.vstack((initial_padding, cumulative_steps))
        for i in range(dims):
            cumulative_steps[:, i] = np.convolve(cumulative_steps_padded[:, i], np.ones(window_size) / window_size,
                                                 'valid')

    return cumulative_steps

if __name__ == "__main__":

    import matplotlib.pyplot as plt

    random_xyz=random_walk(1000,3,step_magnitude=0.1,interval=[],window_size=1,distribution='uniform',order=1.0)
    # radius_min = 0.6
    # radius_max = 1
    # elevation_min= -45
    # elevation_max= 45
    # azimuth_min= -60
    # azimuth_max= 60
    # radius_section = 1
    # elevation_section = 2
    # azimuth_section = 2
    # random_walk = False 
    # sum_section = radius_section * elevation_section * azimuth_section

    # locations=shell_section(np.array([0,0,0]), radius_min=radius_min, radius_max=radius_max, radius_section=radius_section,
    #       elevation_min= elevation_min, elevation_max= elevation_max,elevation_section= elevation_section, azimuth_min= azimuth_min,
    #       azimuth_max = azimuth_max,azimuth_section= azimuth_section, uniform_volume= False)
    # print(locations.shape)
    
    # for location in locations:
    #     print(location)
    
    # location_sections = np.array_split(locations, sum_section/2)

    # for location_section in location_sections:
    #     print(location_section[0])